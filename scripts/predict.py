import argparse
import crtk
import crtk_msgs.msg
from dvrk_teleoperation_ambf import dvrk_teleoperation_ambf, mtm_teleop, psm_teleop, psm_ambf
import math
import numpy
import PyKDL
import std_msgs.msg
import sys


class predict(dvrk_teleoperation_ambf):
    def __init__(self, ral, master, puppet, puppet_virtual, clutch_topic, expected_interval, operator_present_topic="", config_file_name=""):
        super().__init__(ral, master, puppet, puppet_virtual, clutch_topic, expected_interval, operator_present_topic, config_file_name)
        
        self.comm_loss = False
        self.__comm_loss_sub = self.ral.subscriber('/communication_loss',
                                                   std_msgs.msg.Bool,
                                                   self.__comm_loss_cb,
                                                   latch = True)

        self.holding_peg = False
        self.predict = False
        self.predict_vel = 0.01
        self.dist_thresh = 0.01
        self.height_thresh = 0.016
        self.next_goal_ind = 0
        self.peg_to_jaw_transform = PyKDL.Frame()
        self.start_posts = []
        self.end_posts = []
        for i in range(6):
            start_obj = self.ambf_client.get_obj_handle("/ambf/env/Poll_L_" + str(i))
            end_obj = self.ambf_client.get_obj_handle("/ambf/env/Poll_R_" + str(i))
            start_pos = start_obj.get_pos()
            end_pos = end_obj.get_pos()
            self.start_posts.append(self.T_psmbase_c_frame.Inverse() * self.peg_to_jaw_transform * PyKDL.Vector(start_pos.x, start_pos.y, start_pos.z))
            self.end_posts.append(self.T_psmbase_c_frame.Inverse() * self.peg_to_jaw_transform * PyKDL.Vector(end_pos.x, end_pos.y, end_pos.z))
        self.next_goal = self.start_posts[self.next_goal_ind]

    def __comm_loss_cb(self, value):
        if value != None:
            self.comm_loss = value.data
            if self.comm_loss:
                self.predict = (self.puppet_setpoint_cp.p.z() > self.height_thresh)
                if self.predict:
                    self.jaw_pos_loss = self.puppet_jaw_servo_jp
                    self.puppet_rotation_loss = self.master_measured_cp.M * self.alignment_offset_initial
                    self.master.lock_orientation(self.master_measured_cp.M)
                else:
                    # locks master and both puppets (virtual and real)
                    self.operator_present(not self.comm_loss)
                    self.operator_is_present = True
            else:
                if self.predict:
                    self.master_cartesian_initial = self.master_measured_cp
                    self.master.unlock_orientation()
                else:
                    # unlocks master and both puppets (virtual and real)
                    self.operator_present(not self.comm_loss)
                    self.operator_is_present = True

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
                    master_translation = master_position.p - self.master_cartesian_initial.p
                    puppet_translation = master_translation * self.scale
                    puppet_translation = puppet_translation + self.puppet_cartesian_initial.p
                # rotation
                puppet_rotation = PyKDL.Rotation()
                if self.rotation_locked:
                    puppet_rotation = self.puppet_cartesian_initial.M
                else:
                    puppet_rotation = master_position.M * self.alignment_offset_initial

                # desired puppet goal
                puppet_cartesian_goal = PyKDL.Frame(puppet_rotation, puppet_translation)

                if not self.comm_loss:
                    self.puppet.servo_cp(puppet_cartesian_goal)
                elif self.predict:
                    puppet_translation_loss = self.puppet_setpoint_cp.p + (self.next_goal - self.puppet_setpoint_cp.p).Normalize() * self.predict_vel
                    self.puppet.servo_cp(PyKDL.Frame(self.puppet_rotation_loss, puppet_translation_loss))
                else:
                    self.puppet.hold()
                self.puppet_virtual.servo_cp(self.T_psmbase_c_frame * puppet_cartesian_goal)
                
                if not self.jaw_ignore:
                    if callable(getattr(self.master.gripper, "measured_js", None)):
                        self.master_gripper_measured_js = self.master.gripper.measured_js()
                        current_gripper = self.master_gripper_measured_js[0][0]
                        # see if we caught up
                        if not self.jaw_caught_up_after_clutch:
                            error = abs(current_gripper - self.gripper_ghost)
                            if error < self.tolerance_back_from_clutch:
                                self.jaw_caught_up_after_clutch = True
                        # pick rate based on back from clutch or not
                        # TODO: this period can be improved?
                        delta = self.jaw_rate * self.expected_interval if self.jaw_caught_up_after_clutch else self.jaw_rate_back_from_clutch * self.expected_interval
                        if self.gripper_ghost <= (current_gripper - delta):
                            self.gripper_ghost += delta
                        elif self.gripper_ghost >= (current_gripper + delta):
                            self.gripper_ghost -= delta
                        self.puppet_jaw_servo_jp[0] = self.gripper_to_jaw(self.gripper_ghost)
                        # make sure we don't set goal past joint limits
                        if self.puppet_jaw_servo_jp[0] < self.gripper_to_jaw_position_min:
                            self.puppet_jaw_servo_jp[0] = self.gripper_to_jaw_position_min
                            self.gripper_ghost = self.jaw_to_gripper(self.gripper_to_jaw_position_min)
                        
                        if self.comm_loss:
                            self.puppet.jaw.servo_jp(self.jaw_pos_loss)
                            self.puppet_virtual.jaw.servo_jp(self.jaw_pos_loss)
                        else:
                            self.puppet.jaw.servo_jp(self.puppet_jaw_servo_jp)
                            self.puppet_virtual.jaw.servo_jp(self.puppet_jaw_servo_jp)
                        
                        if not self.comm_loss:
                            distance_to_goal = numpy.sqrt(self.next_goal.x * puppet_translation.x + self.next_goal.y * puppet_translation.y)
                            if distance_to_goal < self.dist_thresh and puppet_translation.z < self.height_thresh:
                                if not self.holding_peg and self.puppet_jaw_servo_jp[0] <= math.pi/6:
                                    self.holding_peg = True
                                    self.next_goal = self.end_posts[self.next_goal_ind]
                                elif self.holding_peg and self.puppet_jaw_servo_jp[0] >= math.pi/6:
                                    self.holding_peg = False
                                    self.next_goal_ind = (self.next_goal_ind + 1) % 6
                                    self.next_goal = self.start_posts[self.next_goal_ind]
                    else:
                        self.puppet_jaw_servo_jp[0] = 45 * math.pi / 180
                        self.puppet.jaw.servo_jp(self.puppet_jaw_servo_jp)
                        self.puppet_virtual.jaw.servo_jp(self.puppet_jaw_servo_jp)
    
if __name__ == '__main__':
    # extract ros arguments (e.g. __ns:= for namespace)
    argv = crtk.ral.parse_argv(sys.argv[1:]) # skip argv[0], script name

    # parse arguments
    parser = argparse.ArgumentParser(description = __doc__,
                                     formatter_class = argparse.RawTextHelpFormatter)
    parser.add_argument('-m', '--mtm', type = str, default='MTML', # required = True,
                        choices = ['MTML', 'MTMR'],
                        help = 'MTM arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
    parser.add_argument('-p', '--psm', type = str, default='PSM2', # required = True,
                        choices = ['PSM1', 'PSM2', 'PSM3'],
                        help = 'PSM arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
    parser.add_argument('-c', '--clutch', type = str, default='/footpedals/clutch',
                        help = 'ROS topic corresponding to clutch button/pedal input')
    parser.add_argument('-o', '--operator', type = str, default='/footpedals/coag',
                        help = 'ROS topic corresponding to operator present button/pedal/sensor input')
    parser.add_argument('-i', '--interval', type=float, default=0.005,
                        help = 'expected interval in seconds between messages sent by the device')
    args = parser.parse_args(argv)

    ral = crtk.ral('teleop_predict')
    mtm = mtm_teleop(ral, args.mtm, args.interval)
    psm = psm_teleop(ral, args.psm, args.interval)
    psm_virtual = psm_ambf(ral, '/ambf/env/psm2', args.interval)
    application = predict(ral, mtm, psm, psm_virtual, args.clutch, args.interval, operator_present_topic=args.operator, config_file_name="")
    ral.spin_and_execute(application.run)
