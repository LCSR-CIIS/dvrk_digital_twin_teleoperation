import argparse
from pathlib import Path
import crtk
import crtk_msgs.msg
from dvrk_teleoperation_ambf import (
    dvrk_teleoperation_ambf,
    load_hand_eye_calibration,
    mtm_teleop,
    psm_teleop,
    psm_ambf,
)
import math
import PyKDL
import std_msgs.msg
import sys


class replay(dvrk_teleoperation_ambf):
    def __init__(
        self,
        ral,
        master,
        puppet,
        puppet_virtual,
        clutch_topic,
        expected_interval,
        operator_present_topic="",
        config_file_name="",
        cam_opencv_T_base=None,
    ):
        super().__init__(
            ral,
            master,
            puppet,
            puppet_virtual,
            clutch_topic,
            expected_interval,
            operator_present_topic,
            config_file_name,
            cam_opencv_T_base=cam_opencv_T_base,
        )

        self.comm_loss = False
        self.recording = False
        self.replaying = False
        self.arm_traj = []
        self.jaw_traj = []
        self.__comm_loss_sub = self.ral.subscriber(
            "/communication_loss", std_msgs.msg.Bool, self.__comm_loss_cb, latch=True
        )

    def __comm_loss_cb(self, value):
        if value != None:
            self.comm_loss = value.data
        self.handle_comm_loss(self.comm_loss)

    def handle_comm_loss(self, comm_loss):
        if comm_loss:
            self.recording = True
        else:
            self.replaying = True

    def run_enabled(self):
        if self.master_measured_cp and self.puppet_setpoint_cp:
            if not self.clutched:
                if self.following_master_body_servo_cf:
                    self.master.body.servo_cf(self.following_master_body_servo_cf)
                master_position = self.master_measured_cp
                # translation
                master_translation = PyKDL.Vector()
                puppet_translation = PyKDL.Vector()
                if self.translation_locked:
                    puppet_translation = self.puppet_cartesian_initial.p
                else:
                    master_translation = (
                        master_position.p - self.master_cartesian_initial.p
                    )
                    puppet_translation = master_translation * self.scale
                    puppet_translation = (
                        puppet_translation + self.puppet_cartesian_initial.p
                    )
                # rotation
                puppet_rotation = PyKDL.Rotation()
                if self.rotation_locked:
                    puppet_rotation = self.puppet_cartesian_initial.M
                else:
                    puppet_rotation = master_position.M * self.alignment_offset_initial

                # desired puppet goal
                puppet_cartesian_goal = PyKDL.Frame(puppet_rotation, puppet_translation)

                # TODO: Add PSM base frame (?)
                # TODO: Can't really add velocity to servo_cp?

                self.puppet_virtual.servo_cp(
                    self.T_psmbase_dvrkframe * puppet_cartesian_goal
                )
                if not self.comm_loss:
                    if self.replaying:
                        if len(self.arm_traj) != 0:
                            self.puppet.servo_cp(self.arm_traj[0])
                        if len(self.arm_traj) <= 2:
                            self.arm_traj = []
                            if self.jaw_ignore:
                                self.replaying = False
                                self.recording = False
                        else:
                            self.arm_traj = self.arm_traj[2:]
                    else:
                        self.puppet.servo_cp(puppet_cartesian_goal)
                if self.recording:
                    self.arm_traj.append(puppet_cartesian_goal)

                if not self.jaw_ignore:
                    if callable(getattr(self.master.gripper, "measured_js", None)):
                        try:
                            master_gripper_measured_js = (
                                self.master.gripper.measured_js()
                            )
                            self.master_gripper_measured_js = master_gripper_measured_js
                        except RuntimeWarning as w:
                            print(w)
                        current_gripper = self.master_gripper_measured_js[0][0]
                        # see if we caught up
                        if not self.jaw_caught_up_after_clutch:
                            error = abs(current_gripper - self.gripper_ghost)
                            if error < self.tolerance_back_from_clutch:
                                self.jaw_caught_up_after_clutch = True
                        # pick rate based on back from clutch or not
                        # TODO: this period can be improved?
                        delta = (
                            self.jaw_rate * self.expected_interval
                            if self.jaw_caught_up_after_clutch
                            else self.jaw_rate_back_from_clutch * self.expected_interval
                        )
                        if self.gripper_ghost <= (current_gripper - delta):
                            self.gripper_ghost += delta
                        elif self.gripper_ghost >= (current_gripper + delta):
                            self.gripper_ghost -= delta
                        self.puppet_jaw_servo_jp[0] = self.gripper_to_jaw(
                            self.gripper_ghost
                        )
                        # make sure we don't set goal past joint limits
                        if (
                            self.puppet_jaw_servo_jp[0]
                            < self.gripper_to_jaw_position_min
                        ):
                            self.puppet_jaw_servo_jp[0] = (
                                self.gripper_to_jaw_position_min
                            )
                            self.gripper_ghost = self.jaw_to_gripper(
                                self.gripper_to_jaw_position_min
                            )
                        self.puppet_virtual.jaw.servo_jp(self.puppet_jaw_servo_jp)
                        if not self.comm_loss:
                            if self.replaying:
                                if len(self.jaw_traj) != 0:
                                    self.puppet.jaw.servo_jp(self.jaw_traj[0])
                                if len(self.jaw_traj) <= 2:
                                    self.jaw_traj = []
                                    self.recording = False
                                    self.replaying = False
                                else:
                                    self.jaw_traj = self.jaw_traj[2:]
                            else:
                                self.puppet.jaw.servo_jp(self.puppet_jaw_servo_jp)
                        if self.recording:
                            self.jaw_traj.append(self.puppet_jaw_servo_jp)
                    else:
                        self.puppet_jaw_servo_jp[0] = 45 * math.pi / 180
                        self.puppet_virtual.jaw.servo_jp(self.puppet_jaw_servo_jp)
        return


if __name__ == "__main__":
    # extract ros arguments (e.g. __ns:= for namespace)
    argv = crtk.ral.parse_argv(sys.argv[1:])  # skip argv[0], script name

    # parse arguments
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument(
        "-m",
        "--mtm",
        type=str,
        default="MTML",  # required = True,
        choices=["MTML", "MTMR"],
        help="MTM arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace",
    )
    parser.add_argument(
        "-p",
        "--psm",
        type=str,
        default="PSM2",  # required = True,
        choices=["PSM1", "PSM2", "PSM3"],
        help="PSM arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace",
    )
    parser.add_argument(
        "-c",
        "--clutch",
        type=str,
        default="/footpedals/clutch",
        help="ROS topic corresponding to clutch button/pedal input",
    )
    parser.add_argument(
        "-o",
        "--operator",
        type=str,
        default="/footpedals/coag",
        help="ROS topic corresponding to operator present button/pedal/sensor input",
    )
    parser.add_argument(
        "-i",
        "--interval",
        type=float,
        default=0.005,
        help="expected interval in seconds between messages sent by the device",
    )
    parser.add_argument(
        "-H",
        "--hand-eye-json",
        type=str,
        required=True,
        help="hand-eye calibration matrix in JSON format using OpenCV coordinate system. \
             This is required to set the base of the virtual PSM to match the real setup",
    )

    args = parser.parse_args(argv)

    hand_eye_path = Path(args.hand_eye_json)
    cam_opencv_T_base = load_hand_eye_calibration(hand_eye_path)

    ral = crtk.ral("teleop_replay")
    mtm = mtm_teleop(ral, args.mtm, args.interval)
    psm = psm_teleop(ral, args.psm, args.interval)
    psm_virtual = psm_ambf(ral, "/ambf/env/psm2", 2 * args.interval) ### should be 100 Hz, so times 2
    application = replay(
        ral,
        mtm,
        psm,
        psm_virtual,
        args.clutch,
        args.interval,
        operator_present_topic=args.operator,
        config_file_name="",
        cam_opencv_T_base=cam_opencv_T_base,
    )
    ral.spin_and_execute(application.run)
