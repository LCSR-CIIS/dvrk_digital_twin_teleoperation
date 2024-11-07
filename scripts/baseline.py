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


class baseline(dvrk_teleoperation_ambf):
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
            cam_opencv_T_base,
        )

        self.comm_loss = False
        self.__comm_loss_sub = self.ral.subscriber(
            "/communication_loss", std_msgs.msg.Bool, self.__comm_loss_cb, latch=True
        )

    def __comm_loss_cb(self, value):
        if value != None:
            self.comm_loss = value.data
        self.operator_present(not self.comm_loss)
        self.operator_is_present = True


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

    ral = crtk.ral("teleop_baseline")
    mtm = mtm_teleop(ral, args.mtm, 4 * args.interval)
    psm = psm_teleop(ral, args.psm, 4 * args.interval)
    psm_virtual = psm_ambf(ral, "/ambf/env/psm2", 2 * args.interval)
    application = baseline(
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
