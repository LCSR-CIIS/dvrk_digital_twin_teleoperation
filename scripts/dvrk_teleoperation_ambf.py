from ambf_client import Client
import argparse
import crtk
import crtk_msgs.msg
import dvrk
from enum import Enum
import geometry_msgs.msg
import json
import math
import numpy
import numpy as np
import os
import PyKDL
import rospy
import sensor_msgs.msg
import std_msgs.msg
from surgical_robotics_challenge.kinematics.psmKinematics import PSMType, ToolType
from surgical_robotics_challenge.kinematics.psmKinematics import PSMKinematicSolver
import sys
import time
import tf_conversions.posemath as pm

# Global kinematic solver
psm_solver = PSMKinematicSolver(
    psm_type=PSMType.LND_SI.value, tool_id=ToolType.LND_SI.value
)

class dvrk_teleoperation_ambf:
    class state(Enum):
        DISABLED = 1
        SETTING_ARMS_STATE = 2
        ALIGNING_ARM = 3
        ENABLED = 4

    class clutch_button(crtk.joystick_button):
        def __init__(self, ral, clutch_topic, teleop_class):
            super().__init__(ral, clutch_topic)
            self.set_callback(self.clutch_cb)
            self.teleop_class = teleop_class

        def clutch_cb(self, value):
            if value != None:
                self.teleop_class.clutched = value
            if self.teleop_class.desired_state == self.teleop_class.state.ENABLED:
                self.teleop_class.clutch(self.teleop_class.clutched)

    class operator_present_button(crtk.joystick_button):
        def __init__(self, ral, operator_present_topic, teleop_class):
            super().__init__(ral, operator_present_topic)
            self.set_callback(self.operator_present_cb)
            self.teleop_class = teleop_class

        def operator_present_cb(self, value):
            if value != None:
                self.teleop_class.operator_is_present = value
            self.teleop_class.operator_present(self.teleop_class.operator_is_present)

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
    ):
        print(
            "Initialzing dvrk_teleoperation for {} and {}".format(
                master.name(), puppet.name()
            )
        )
        self.ral = ral
        self.expected_interval = expected_interval

        self.master = master
        # Required features of master
        getattr(self.master, "measured_cp")
        getattr(self.master, "setpoint_cp")
        getattr(self.master, "move_cp")
        getattr(self.master, "body")
        getattr(self.master.body, "servo_cf")
        getattr(self.master, "use_gravity_compensation")
        getattr(self.master, "operating_state")
        getattr(self.master, "state_command")

        self.puppet = puppet
        # Required features of puppet
        getattr(self.puppet, "setpoint_cp")
        getattr(self.puppet, "servo_cp")
        getattr(self.puppet, "hold")
        getattr(self.puppet, "operating_state")
        getattr(self.puppet, "state_command")

        self.puppet_virtual = puppet_virtual
        # Required features of virtual puppet
        getattr(self.puppet_virtual, "setpoint_cp")
        getattr(self.puppet_virtual, "servo_cp")

        self.clutch_obj = self.clutch_button(ral, clutch_topic, self)
        if operator_present_topic != "":
            self.operator_present_obj = self.operator_present_button(
                ral, operator_present_topic, self
            )
            self.operator_is_present = False
        else:
            self.operator_is_present = True  # if not given, then always assume present

        self.following = False
        self.clutched = False
        self.back_from_clutch = False
        self.jaw_caught_up_after_clutch = False
        self.rotation_locked = False
        self.translation_locked = False
        self.align_master = True
        self.scale = 0.2
        self.jaw_ignore = False
        self.jaw_rate = 4 * math.pi
        self.jaw_rate_back_from_clutch = 0.2 * math.pi
        self.gripper_max = 60 * math.pi / 180
        self.gripper_zero = 0
        self.jaw_min = -20 * math.pi / 180
        self.jaw_max = 80 * math.pi / 180
        self.operator_orientation_tolerance = 5 * math.pi / 180
        self.operator_gripper_threshold = 5 * math.pi / 180
        self.operator_roll_threshold = 3 * math.pi / 180
        self.operator_is_active = False
        self.tolerance_back_from_clutch = 2 * math.pi / 180
        self.master_use_measured_cv = False

        self.master_measured_cp = PyKDL.Frame()
        self.master_move_cp = PyKDL.Frame()
        self.puppet_setpoint_cp = PyKDL.Frame()
        self.puppet_servo_cp = PyKDL.Frame()
        self.puppet_jaw_servo_jp = numpy.zeros((1,))
        self.puppet_virtual_setpoint_cp = PyKDL.Frame()
        self.puppet_virtual_servo_cp = PyKDL.Frame()
        self.puppet_virtual_jaw_servo_jp = numpy.zeros((1,))

        self.__state_command_sub = self.ral.subscriber(
            "/teleop_python/state_command",
            crtk_msgs.msg.StringStamped,
            self.__state_command_cb,
            latch=True,
        )

        if config_file_name != "":
            self.configure(config_file_name)

        self.current_state = self.state.DISABLED
        self.desired_state = self.state.DISABLED
        self.from_disabled = True

        self.running = True
        self.sleep_rate = self.ral.create_rate(int(1 / expected_interval))

        self.ambf_client = Client()
        self.ambf_client.connect()

        self.startup()

    def configure(self, file_name):
        try:
            script_dir = os.path.dirname(__file__)
            file_pointer = open(os.path.join(script_dir, file_name))
        except OSError as error:
            print("Failed to load configuration file:", error)
            return

        try:
            json_config = json.load(file_pointer)
        except Exception as error:
            print("Error while parsing JSON:", error)
            return

        # read scale if present
        try:
            json_value = json_config["scale"]
            self.scale = float(json_value)
            if self.scale <= 0.0:
                print(
                    f'Configure {self.ral.node_name()}: "scale" must be a positive number. Found {self.scale}'
                )
                raise ValueError
        except KeyError:
            pass

        # read orientation if present
        try:
            json_value = json_config["rotation"]
            print(f'Configure {self.ral.node_name()}: "rotation" is deprecated.')
            raise ValueError
        except KeyError:
            pass

        # rotation locked
        try:
            json_value = json_config["rotation-locked"]
            self.rotation_locked = bool(json_value)
        except KeyError:
            pass

        # translation locked
        try:
            json_value = json_config["translation-locked"]
            self.translation_locked = bool(json_value)
        except KeyError:
            pass

        # ignore jaw if needed
        try:
            json_value = json_config["ignore-jaw"]
            self.jaw_ignore = bool(json_value)
        except KeyError:
            pass

        # jaw rate of opening-closing
        try:
            json_value = json_config["jaw-rate"]
            self.jaw_rate = float(json_value)
            if self.jaw_rate <= 0.0:
                print(
                    f'Configure {self.ral.node_name()}: "jaw-rate" must be a positive number. Found {self.jaw_rate}'
                )
                raise ValueError
        except KeyError:
            pass

        # jaw rate of opening-closing after clutch
        try:
            json_value = json_config["jaw-rate-back-from-clutch"]
            self.jaw_rate_back_from_clutch = float(json_value)
            if self.jaw_rate_back_from_clutch <= 0.0:
                print(
                    f'Configure {self.ral.node_name()}: "jaw-rate-back-from-clutch" must be a positive number. Found {self.jaw_rate_back_from_clutch}'
                )
                raise ValueError
        except KeyError:
            pass

        # gripper scaling
        try:
            json_gripper = json_config["gripper-scaling"]
            try:
                json_value = json_gripper["max"]
                self.gripper_max = float(json_value)
            except KeyError:
                print(
                    f'Configure {self.ral.node_name()}: "gripper-scaling":  {{"max"}} is missing.'
                )
                raise ValueError
            try:
                json_value = json_gripper["zero"]
                self.gripper_zero = float(json_value)
            except KeyError:
                print(
                    f'Configure {self.ral.node_name()}: "gripper-scaling":  {{"zero"}} is missing.'
                )
                raise ValueError
        except KeyError:
            pass

        # orientation tolerance to start teleop
        try:
            json_value = json_config["start-orientation-tolerance"]
            self.operator_orientation_tolerance = float(json_value)
            if self.operator_orientation_tolerance < 0.0:
                print(
                    f'Configure {self.ral.node_name()}: "start-orientation-tolerance" must be a positive number. Found {self.operator_orientation_tolerance}'
                )
                raise ValueError
        except KeyError:
            pass

        # gripper threshold to start teleop
        try:
            json_value = json_config["start-gripper-threshold"]
            self.operator_gripper_threshold = float(json_value)
            if self.operator_gripper_threshold < 0.0:
                print(
                    f'Configure {self.ral.node_name()}: "start-gripper-threshold" must be a positive number. Found {self.operator_gripper_threshold}'
                )
                raise ValueError
        except KeyError:
            pass

        # roll threshold to start teleop
        try:
            json_value = json_config["start-roll-threshold"]
            self.operator_roll_threshold = float(json_value)
            if self.operator_roll_threshold < 0.0:
                print(
                    f'Configure {self.ral.node_name()}: "start-roll-threshold" must be a positive number. Found {self.operator_roll_threshold}'
                )
                raise ValueError
        except KeyError:
            pass

        # align MTM if needed
        try:
            json_value = json_config["align-mtm"]
            self.align_master = bool(json_value)
        except KeyError:
            pass

        # use MTM cv and send to PSM
        self.master_use_measured_cv = True
        try:
            json_value = json_config["use_mtm_velocity"]
            self.master_use_measured_cv = bool(json_value)
        except KeyError:
            pass

    def startup(self):
        self.set_scale(self.scale)
        self.set_following(self.following)
        self.lock_rotation(self.rotation_locked)
        self.lock_translation(self.translation_locked)
        self.set_align_master(self.align_master)

        # check if functions for jaw are connected
        if not self.jaw_ignore:
            if not callable(
                getattr(self.puppet.jaw, "setpoint_js", None)
            ) or not callable(getattr(self.puppet.jaw, "servo_jp", None)):
                print(
                    f'{self.ral.node_name()}: optional functions "jaw/servo_jp" and "jaw/setpoint_js" are not connected for {self.puppet.name()}, setting "ignore-jaw" to true'
                )
                self.jaw_ignore = True
            if not callable(
                getattr(self.puppet_virtual.jaw, "setpoint_js", None)
            ) or not callable(getattr(self.puppet_virtual.jaw, "servo_jp", None)):
                print(
                    f'{self.ral.node_name()}: optional functions "jaw/servo_jp" and "jaw/setpoint_js" are not connected for {self.puppet_virtual.name()}, setting "ignore-jaw" to true'
                )
                self.jaw_ignore = True

        # check if MTM has measured_cv as needed
        if self.master_use_measured_cv and not callable(
            getattr(self.master, "measured_cv", None)
        ):
            self.master_use_measured_cv = False
            print(
                f'{self.ral.node_name()}: master ({self.master.name()} doesn\'t provide measured_cv, you can avoid this warning by setting "use-mtm-velocity" to false)'
            )

        ##############
        # AMBF SETUP 
        ##############

        self.new_ambf_setup()

        # cameraframe_obj = self.ambf_client.get_obj_handle("/CameraFrame")
        # cameraleft_obj = self.ambf_client.get_obj_handle("/ambf/env/cameras/cameraL")
        # # cameraright_obj = self.ambf_client.get_obj_handle("/ambf/env/cameras/cameraR")
        # self.psmtool_obj = self.ambf_client.get_obj_handle("/ambf/env/psm2/toolyawlink")
        # self.psmbase_obj = self.ambf_client.get_obj_handle("/ambf/env/psm2/baselink")

        self.psmbase_pub = rospy.Publisher(
            "ambf_psmbase", geometry_msgs.msg.PoseStamped, queue_size=10
        )
        self.psmtool_pub = rospy.Publisher(
            "ambf_psmtool", geometry_msgs.msg.PoseStamped, queue_size=10
        )
        self.cameraframe_pub = rospy.Publisher(
            "ambf_cameraframe", geometry_msgs.msg.PoseStamped, queue_size=10
        )
        self.cameraleft_pub = rospy.Publisher(
            "ambf_cameraleft", geometry_msgs.msg.PoseStamped, queue_size=10
        )


        # # T_c_psmbase = rospy.wait_for_message(f'/SUJ/{self.puppet.name()}/measured_cp', geometry_msgs.msg.PoseStamped)
        # T_w_psmbase = rospy.wait_for_message(
        #     f"/SUJ/{self.puppet.name()}/local/measured_cp",
        #     geometry_msgs.msg.PoseStamped,
        # )
        # psm_fk = rospy.wait_for_message(
        #     f"/{self.puppet.name()}/local/measured_cp", geometry_msgs.msg.PoseStamped
        # )
        # # psm_js = rospy.wait_for_message(f'/{self.puppet.name()}/measured_js', sensor_msgs.msg.JointState)
        # psm_jaw_js = rospy.wait_for_message(
        #     f"/{self.puppet.name()}/jaw/measured_js", sensor_msgs.msg.JointState
        # )

        ## HAND-EYE CALIBRATION - Add opencv hand-eye
        # TODO: update this after each ArUco calibration (with left camera)
        #fmt: off
        # T_left_psmbase = [[-0.8790781792313629, 0.3412618733306592, -0.3328090873310377, -0.08238268273316497], 
        #                      [0.33239054126068474, 0.9392881778897073, 0.08517186716905183, -0.007596158536450624], 
        #                      [0.34166955216948763, -0.03574986276173252, -0.9391399599808413, 0.09819970244543609], 
        #                      [0.0, 0.0, 0.0, 1.0]]
        # T_left_psmbase = np.array(T_left_psmbase)
        # T_left_psmbase = pm.fromMatrix(T_left_psmbase)
        #fmt :on

        # T_left_psmbase = PyKDL.Frame(
        #     PyKDL.Rotation(
        #         -0.425680413343186, 0.4255129190992243, -0.7985830835771751,
        #         0.6500765197494777, 0.7577116318433726, 0.05721539512805919,
        #         0.6294415812181297, -0.4947846386549507, -0.5991589581944915,),
        #     PyKDL.Vector(
        #         -0.06643774227587187, -0.03572082293958062, -0.021547666872504815
        #     ),
        # )

        # # TODO: update this after each ArUco calibration (with right camera)
        # T_right_psmbase = PyKDL.Frame(
        #     PyKDL.Rotation(1, 0, 0, 0, 1, 0, 0, 0, 1), PyKDL.Vector(0, 0, 0)
        # )


        #############################################################################################
        # compute the "mean" between left and right cameras to put the camera frame
        # R_left_psmbase = np.array(
        #     [
        #         [T_left_psmbase[0, 0], T_left_psmbase[0, 1], T_left_psmbase[0, 2]],
        #         [T_left_psmbase[1, 0], T_left_psmbase[1, 1], T_left_psmbase[1, 2]],
        #         [T_left_psmbase[2, 0], T_left_psmbase[2, 1], T_left_psmbase[2, 2]],
        #     ]
        # )
        # R_right_psmbase = np.array(
        #     [
        #         [T_right_psmbase[0, 0], T_right_psmbase[0, 1], T_right_psmbase[0, 2]],
        #         [T_right_psmbase[1, 0], T_right_psmbase[1, 1], T_right_psmbase[1, 2]],
        #         [T_right_psmbase[2, 0], T_right_psmbase[2, 1], T_right_psmbase[2, 2]],
        #     ]
        # )
        # U, _, Vh = np.linalg.svd((R_left_psmbase + R_right_psmbase) / 2)
        # R_cameraframe_psmbase = U @ Vh
        # #fmt: off
        # T_cameraframe_psmbase = PyKDL.Frame(
        #     PyKDL.Rotation(
        #         R_cameraframe_psmbase[0, 0], R_cameraframe_psmbase[0, 1], R_cameraframe_psmbase[0, 2],
        #         R_cameraframe_psmbase[1, 0], R_cameraframe_psmbase[1, 1], R_cameraframe_psmbase[1, 2],
        #         R_cameraframe_psmbase[2, 0], R_cameraframe_psmbase[2, 1], R_cameraframe_psmbase[2, 2],
        #     ),
        #     (T_left_psmbase.p + T_right_psmbase.p) / 2,
        # )
        # #fmt: on
        #############################################################################################


        #############################################################################################
        ### Set the virtual camera to mirror the real system readings (Not needed if handeye is used)

        # T_w_c_real = rospy.wait_for_message( "/ECM/measured_cp", geometry_msgs.msg.PoseStamped)
        # T_w_cameraframe = crtk.msg_conversions.FrameFromPoseMsg(T_w_c_real.pose)
        # cameraframe_obj.set_pos(
        #     T_w_c_real.pose.position.x, 
        #     T_w_c_real.pose.position.y, 
        #     T_w_c_real.pose.position.z,
        # )
        # cameraframe_obj.set_rot( [
        #         T_w_c_real.pose.orientation.x,
        #         T_w_c_real.pose.orientation.y,
        #         T_w_c_real.pose.orientation.z,
        #         T_w_c_real.pose.orientation.w,
        #     ]
        # )

        #############################################################################################

        #############################################################################################
        # Sets the virtual PSM base frame based on the computed mean
        # T_w_psmbase = crtk.msg_conversions.FrameToPoseMsg(
        #     T_w_cameraframe * T_cameraframe_psmbase
        # )
        # self.psmbase_obj.set_pos(
        #     T_w_psmbase.position.x, T_w_psmbase.position.y, T_w_psmbase.position.z
        # )
        # self.psmbase_obj.set_rot(
        #     [
        #         T_w_psmbase.orientation.x,
        #         T_w_psmbase.orientation.y,
        #         T_w_psmbase.orientation.z,
        #         T_w_psmbase.orientation.w,
        #     ]
        # )
        #############################################################################################

        # T_w_c_virtual_frame = crtk.msg_conversions.FrameFromPoseMsg(T_w_c_real.pose)
        # cameraframe_obj.set_pos(T_w_c_real.pose.position.x, T_w_c_real.pose.position.y, T_w_c_real.pose.position.z)
        # cameraframe_obj.set_rot([T_w_c_real.pose.orientation.x, T_w_c_real.pose.orientation.y, T_w_c_real.pose.orientation.z, T_w_c_real.pose.orientation.w])

        # hard-coded from hand-eye calibration
        # T_ext = PyKDL.Frame(PyKDL.Rotation(-0.99875061, 0.03992935, -0.030047648,
        #                                    -0.039027081, -0.99878656, -0.030038183,
        #                                    -0.031210593, -0.028827982, 0.99909702),
        #                     PyKDL.Vector(0.0037492396, 0.011333806, -0.016384585))

        # corrects for OpenCV / AMBF convensions
        # T_cv_ambf = PyKDL.Frame(
        #     PyKDL.Rotation(0, 1, 0, 0, 0, -1, -1, 0, 0), PyKDL.Vector()
        # )
        # # sets left and right cameras
        # T_cf_cl = crtk.msg_conversions.FrameToPoseMsg(
        #     T_cameraframe_psmbase * T_left_psmbase.Inverse() * T_cv_ambf
        # )
        # cameraleft_obj.set_pos(
        #     T_cf_cl.position.x, T_cf_cl.position.y, T_cf_cl.position.z
        # )
        # cameraleft_obj.set_rot(
        #     [
        #         T_cf_cl.orientation.x,
        #         T_cf_cl.orientation.y,
        #         T_cf_cl.orientation.z,
        #         T_cf_cl.orientation.w,
        #     ]
        # )
        # T_cf_cr = crtk.msg_conversions.FrameToPoseMsg(
        #     T_cameraframe_psmbase * T_right_psmbase.Inverse() * T_cv_ambf
        # )
        # cameraleft_obj.set_pos(
        #     T_cf_cr.position.x, T_cf_cr.position.y, T_cf_cr.position.z
        # )
        # cameraleft_obj.set_rot(
        #     [
        #         T_cf_cr.orientation.x,
        #         T_cf_cr.orientation.y,
        #         T_cf_cr.orientation.z,
        #         T_cf_cr.orientation.w,
        #     ]
        # )

        # T_w_psmbase = crtk.msg_conversions.FrameToPoseMsg(T_w_c_virtual_frame * T_ext * T_left_psmbase)
        # self.psmbase_obj.set_pos(T_w_psmbase.position.x, T_w_psmbase.position.y, T_w_psmbase.position.z)
        # self.psmbase_obj.set_rot([T_w_psmbase.orientation.x, T_w_psmbase.orientation.y, T_w_psmbase.orientation.z, T_w_psmbase.orientation.w])

        # self.T_psmbase_c_frame = T_cameraframe_psmbase.Inverse()
        # self.puppet_virtual_servo_cp = crtk.msg_conversions.FrameFromPoseMsg(
        #     psm_fk.pose
        # )
        # self.puppet_virtual.servo_cp(self.puppet_virtual_servo_cp)
        # self.puppet_virtual.jaw.servo_jp(psm_jaw_js.position)


        # # Testing
        # self.cameraleft_obj = cameraleft_obj
        # self.cameraframe_obj = cameraframe_obj

        print("Initialization finished")
    
    def new_ambf_setup(self):
        print("AMBF initialization started")

        camera_frame_handle = self.ambf_client.get_obj_handle("/CameraFrame")
        cameraleft_obj = self.ambf_client.get_obj_handle("/ambf/env/cameras/cameraL")
        # cameraright_obj = self.ambf_client.get_obj_handle("/ambf/env/cameras/cameraR")
        self.psmtool_obj = self.ambf_client.get_obj_handle("/ambf/env/psm2/toolyawlink")
        self.psmbase_obj = self.ambf_client.get_obj_handle("/ambf/env/psm2/baselink")
        # Testing
        self.cameraleft_obj = cameraleft_obj
        self.cameraframe_obj = camera_frame_handle 

        ##################################
        ## step one: set camera frame pose (copied from full_ar_pipeline script)

        # fmt: off
        # TODO: update after hand-eye (juan)
        # cam_opencv_T_base = [[-0.8790781792313629, 0.3412618733306592, -0.3328090873310377, -0.08238268273316497], 
        #                      [0.33239054126068474, 0.9392881778897073, 0.08517186716905183, -0.007596158536450624], 
        #                      [0.34166955216948763, -0.03574986276173252, -0.9391399599808413, 0.09819970244543609], 
        #                      [0.0, 0.0, 0.0, 1.0]]
        cam_opencv_T_base = [[-0.5459032453444352, -0.7436337668137282, -0.38600319632982144, -0.05864828177066425], 
                              [-0.5960559982739572, 0.6684655430310895, -0.4448270053647678, -0.10923058143918474], 
                              [0.5888182178262232, -0.012752985320163394, -0.8081648765699816, 0.014914654894347932],
                              [0.0, 0.0, 0.0, 1.0]]
        # fmt: on

        cam_opencv_T_base = np.array(cam_opencv_T_base)
        base_T_cam_opencv = np.linalg.inv(cam_opencv_T_base) 

        # fmt: off
        cam_opencv_T_cam_ambf = [[ 0, 1,  0, 0], 
                                [ 0, 0, -1, 0], 
                                [-1, 0,  0, 0], 
                                [ 0, 0,  0, 1]]
        cam_opencv_T_cam_ambf = np.array(cam_opencv_T_cam_ambf)

        cam_ambf_T_cam_frame = [[0, 0, 1, 0],
                                [1, 0, 0, 0],
                                [0, 1, 0, 0],
                                [0, 0, 0, 1]]
        cam_ambf_T_cam_frame = np.array(cam_ambf_T_cam_frame)
        
        # fmt: on

        base_T_cam_frame = base_T_cam_opencv @ cam_opencv_T_cam_ambf @ cam_ambf_T_cam_frame
        cam_frame_T_base = np.linalg.inv(base_T_cam_frame) # Virtual PSM base frame.

        camera_frame_handle.set_pose(pm.toMsg(pm.fromMatrix(base_T_cam_frame)))

        ################################
        ## step two: set teleoperation base frame for virtual PSM 

        #fmt: off
        dvrk_frame_T_cam_frame = [[-1, 0, 0, 0],
                                  [ 0, 1, 0, 0],
                                  [ 0, 0,-1, 0],
                                  [ 0, 0, 0, 1]]
        dvrk_frame_T_cam_frame = np.array(dvrk_frame_T_cam_frame)
        #fmt: on
        T_cameraframe_psmbase = dvrk_frame_T_cam_frame @ cam_frame_T_base 
        T_cameraframe_psmbase = pm.fromMatrix(T_cameraframe_psmbase) # Virtual PSM base frame
        self.T_psmbase_c_frame = T_cameraframe_psmbase.Inverse()

        ################################
        ## step Three: set instrument pose

        psm_fk = rospy.wait_for_message(
            f"/{self.puppet.name()}/local/measured_cp", geometry_msgs.msg.PoseStamped
        )
        # psm_js = rospy.wait_for_message(f'/{self.puppet.name()}/measured_js', sensor_msgs.msg.JointState)
        psm_jaw_js = rospy.wait_for_message(
            f"/{self.puppet.name()}/jaw/measured_js", sensor_msgs.msg.JointState
        )
        self.puppet_virtual_servo_cp = crtk.msg_conversions.FrameFromPoseMsg(
            psm_fk.pose
        )
        self.puppet_virtual.servo_cp(self.puppet_virtual_servo_cp)
        self.puppet_virtual.jaw.servo_jp(psm_jaw_js.position)
        
        print("AMBF initialization finished")

    def run_all_states(self):
        self.master_measured_cp = self.master.measured_cp()
        if self.master_use_measured_cv:
            self.master_measured_cv = self.master.measured_cv()
        self.master_setpoint_cp = self.master.setpoint_cp()
        self.puppet_setpoint_cp = self.puppet.setpoint_cp()

        # try:
        #     puppet_virtual_setpoint_cp = self.puppet_virtual.setpoint_cp()
        #     self.puppet_virtual_setpoint_cp = puppet_virtual_setpoint_cp
        # except RuntimeWarning as w:
        #     print(w)

        # TODO: add base frame (?)

        if (
            self.desired_state == self.state.DISABLED
            and self.current_state != self.state.DISABLED
        ):
            self.set_following(False)
            self.set_current_state(self.state.DISABLED)

        if (
            self.current_state != self.state.DISABLED
            and self.current_state != self.state.SETTING_ARMS_STATE
        ):
            if not self.puppet.is_enabled() or not self.puppet.is_homed():
                self.set_desired_state(self.state.DISABLED)
                print(
                    f'ERROR: {self.ral.node_name()}: puppet ({self.puppet.name()}) is not in state "ENABLED" anymore'
                )
            if not self.master.is_enabled() or not self.master.is_homed():
                self.set_desired_state(self.state.DISABLED)
                print(
                    f'ERROR: {self.ral.node_name()}: puppet ({self.master.name()}) is not in state "READY" anymore'
                )

        # Testing only
        T_w_psmtool_pos = self.psmtool_obj.get_pos()
        T_w_psmtool_rot = self.psmtool_obj.get_rot()

        T_w_psmbase_pos = self.psmbase_obj.get_pos()
        T_w_psmbase_rot = self.psmbase_obj.get_rot()

        T_w_cf_pos = self.cameraframe_obj.get_pos()
        T_w_cf_rot = self.cameraframe_obj.get_rot()

        T_cf_cl_pos = self.cameraleft_obj.get_pos()
        T_cf_cl_rot = self.cameraleft_obj.get_rot()

        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = "Cart"
        p.header.stamp = rospy.Time.now()
        p.pose.position = T_w_cf_pos
        p.pose.orientation = T_w_cf_rot
        self.cameraframe_pub.publish(p)

        p.header.frame_id = "ECM"
        p.header.stamp = rospy.Time.now()
        p.pose.position = T_cf_cl_pos
        p.pose.orientation = T_cf_cl_rot
        self.cameraleft_pub.publish(p)

        # p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = "Cart"
        p.header.stamp = rospy.Time.now()
        p.pose.position = T_w_psmtool_pos
        p.pose.orientation = T_w_psmtool_rot
        self.psmtool_pub.publish(p)

        p.header.stamp = rospy.Time.now()
        p.pose.position = T_w_psmbase_pos
        p.pose.orientation = T_w_psmbase_rot
        self.psmbase_pub.publish(p)

        self.sleep_rate.sleep()

    def transition_disabled(self):
        if self.current_state != self.desired_state:
            self.set_current_state(self.state.SETTING_ARMS_STATE)
            self.entering_state = True
            self.from_disabled = True

    def enter_setting_arms_state(self):
        self.in_state_timer = time.perf_counter()

        if not self.puppet.is_enabled():
            self.puppet.enable()
        if not self.puppet.is_homed():
            self.puppet.home()

        if not self.master.is_enabled():
            self.master.enable()
        if not self.master.is_homed():
            self.master.home()

    def transition_setting_arms_state(self):
        if (
            self.puppet.is_enabled()
            and self.puppet.is_homed()
            and self.master.is_enabled()
            and self.master.is_homed()
        ):
            if self.from_disabled and self.align_master:
                input("Press [enter] after master arm has been homed to start aligning")
                self.from_disabled = False
            self.set_current_state(self.state.ALIGNING_ARM)
            self.entering_state = True
            return
        if time.perf_counter() - self.in_state_timer > 60:
            if not (self.puppet.is_enabled() and self.puppet.is_homed()):
                print(
                    f"ERROR: {self.ral.node_name()} timed out when setting up puppet ({self.puppet.name()}) state"
                )
            if not (self.master.is_enabled() and self.master.is_homed()):
                print(
                    f"ERROR: {self.ral.node_name()} timed out when setting up master ({self.master.name()}) state"
                )
            self.set_desired_state(self.state.DISABLED)

    def enter_aligning_arm(self):
        # TODO: Update GUI to reflect scale?
        # reset timer
        self.in_state_timer = time.perf_counter()
        self.time_since_last_align = 0

        if not self.align_master:
            self.master_move_cp = self.master_setpoint_cp
            self.master.move_cp(self.master_move_cp)

        if self.back_from_clutch:
            self.operator_is_active = self.operator_was_active_before_clutch
            self.back_from_clutch = False

        if not self.jaw_ignore:
            self.update_gripper_to_jaw_configuration()

        self.operator_roll_min = math.pi * 100
        self.operator_roll_max = -math.pi * 100
        self.operator_gripper_min = math.pi * 100
        self.operator_gripper_max = -math.pi * 100

    def transition_aligning_arm(self):
        if self.desired_state == self.current_state:
            return

        desired_orientation = self.update_align_offset()
        orientation_error = 0
        # set error only if we need to align MTM to PSM
        if self.align_master:
            orientation_error, _ = self.alignment_offset.GetRotAngle()

        # if not active, use gripper and/or roll to detect if the user is ready
        if not self.operator_is_active:
            gripper_range = 0
            if callable(getattr(self.master.gripper, "measured_js", None)):
                self.master_gripper_measured_js = self.master.gripper.measured_js()
                gripper = self.master_gripper_measured_js[0][0]
                if gripper > self.operator_gripper_max:
                    self.operator_gripper_max = gripper
                elif gripper < self.operator_gripper_min:
                    self.operator_gripper_min = gripper
                gripper_range = self.operator_gripper_max - self.operator_gripper_min

            # checking roll
            roll = math.acos(
                PyKDL.dot(
                    PyKDL.Vector(
                        desired_orientation[0, 1],
                        desired_orientation[1, 1],
                        desired_orientation[2, 1],
                    ),
                    PyKDL.Vector(
                        self.master_measured_cp.M[0, 1],
                        self.master_measured_cp.M[1, 1],
                        self.master_measured_cp.M[2, 1],
                    ),
                )
            )
            if roll > self.operator_roll_max:
                self.operator_roll_max = roll
            elif roll < self.operator_gripper_min:
                self.operator_roll_min = roll
            roll_range = self.operator_roll_max - self.operator_roll_min

            if gripper_range >= self.operator_gripper_threshold:
                self.operator_is_active = True
                print(f"Made active by gripper: {gripper_range}")
            elif roll_range >= self.operator_roll_threshold:
                self.operator_is_active = True
                print(f"Made active by roll: {roll_range}")
            elif gripper_range + roll_range > 0.8 * (
                self.operator_gripper_threshold + self.operator_roll_threshold
            ):
                self.operator_is_active = True
                print(f"Made active by combination: {gripper_range} + {roll_range}")

        # Check for actual transition
        if (
            orientation_error <= self.operator_orientation_tolerance
        ) and self.operator_is_active:
            if self.desired_state == self.state.ENABLED:
                self.set_current_state(self.state.ENABLED)
                self.entering_state = True
        else:
            if time.perf_counter() - self.in_state_timer > 2:
                if orientation_error >= self.operator_orientation_tolerance:
                    print(
                        f"{self.ral.node_name()}: unable to align master ({self.master.name()}), angle error is {orientation_error * 180 / math.pi} (deg)"
                    )
                elif not self.operator_is_active:
                    print(
                        f"{self.ral.node_name()}: pinch/twist master ({self.master.name()}) gripper a bit"
                    )
                self.in_state_timer = time.perf_counter()

    def run_aligning_arm(self):
        # Run
        if self.clutched or not self.align_master:
            return

        current_time = time.perf_counter()
        if current_time - self.time_since_last_align > 10:
            self.time_since_last_align = current_time
            master_cartesian_goal = PyKDL.Frame()
            master_cartesian_goal.p = self.master_setpoint_cp.p
            master_cartesian_goal.M = self.puppet_setpoint_cp.M
            self.master_move_cp = master_cartesian_goal
            handle = self.master.move_cp(self.master_move_cp)
            handle.wait()

    def enter_enabled(self):
        # update MTM/PSM previous position
        self.update_initial_state()

        # set gripper ghost if needed
        if not self.jaw_ignore:
            self.jaw_caught_up_after_clutch = False
            # gripper ghost
            self.puppet_jaw_setpoint_js = self.puppet.jaw.setpoint_js()
            if len(self.puppet_jaw_setpoint_js[0]) != 1:
                print(
                    f"{self.ral.node_name()}: unable to get jaw position. Make sure there is an instrument on the puppet ({self.puppet.name()})"
                )
                self.set_desired_state(self.state.DISABLED)
            current_jaw = self.puppet_jaw_setpoint_js[0][0]
            self.gripper_ghost = self.jaw_to_gripper(current_jaw)

        # set MTM/PSM to Teleop (Cartesian Position Mode)
        self.master.use_gravity_compensation(True)
        # set forces to zero and lock/unlock orientation as needed
        wrench = [0, 0, 0, 0, 0, 0]
        self.master.body.servo_cf(wrench)
        # reset user wrench
        self.following_master_body_servo_cf = wrench

        # orientation locked or not
        if self.rotation_locked and callable(
            getattr(self.master, "lock_orientation", None)
        ):
            self.master.lock_orientation(self.master_measured_cp.M)
        elif callable(getattr(self.master, "unlock_orientation", None)):
            self.master.unlock_orientation()

        # check if by any chance the clutch pedal is pressed
        if self.clutched:
            self.clutch(True)
        else:
            self.set_following(True)

    def transition_enabled(self):
        if self.desired_state != self.current_state:
            self.set_following(False)
            self.set_current_state(self.desired_state)

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

                self.puppet.servo_cp(puppet_cartesian_goal)
                self.puppet_virtual.servo_cp(
                    self.T_psmbase_c_frame * puppet_cartesian_goal
                )

                if not self.jaw_ignore:
                    if callable(getattr(self.master.gripper, "measured_js", None)):
                        self.master_gripper_measured_js = (
                            self.master.gripper.measured_js()
                        )
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
                        self.puppet.jaw.servo_jp(self.puppet_jaw_servo_jp)
                        self.puppet_virtual.jaw.servo_jp(self.puppet_jaw_servo_jp)
                        # print(f"Jaw rate selected: {delta/self.expected_interval}")
                    else:
                        self.puppet_jaw_servo_jp[0] = 45 * math.pi / 180
                        self.puppet.jaw.servo_jp(self.puppet_jaw_servo_jp)
                        self.puppet_virtual.jaw.servo_jp(self.puppet_jaw_servo_jp)

    def clutch(self, clutch):
        if clutch:
            # keep track of last follow mode
            self.operator_was_active_before_clutch = self.operator_is_active
            self.set_following(False)
            self.master_move_cp.M = self.puppet_setpoint_cp.M
            self.master_move_cp.p = self.master_measured_cp.p

            wrench = [0, 0, 0, 0, 0, 0]
            self.master.body.servo_cf(wrench)
            self.master.use_gravity_compensation(True)
            if (self.align_master or self.rotation_locked) and callable(
                getattr(self.master, "lock_orientation", None)
            ):
                self.master.lock_orientation(self.master_measured_cp.M)
            elif callable(getattr(self.master, "unlock_orientation", None)):
                self.master.unlock_orientation()

            self.puppet.hold()
            self.puppet_virtual.hold()
        else:
            self.set_current_state(self.state.SETTING_ARMS_STATE)
            self.back_from_clutch = True
            self.jaw_caught_up_after_clutch = False

    def operator_present(self, operator_is_present):
        if not operator_is_present:
            self.set_desired_state(self.state.ALIGNING_ARM)
        else:
            self.set_desired_state(self.state.ENABLED)

    def update_align_offset(self):
        desired_orientation = self.puppet_setpoint_cp.M
        self.alignment_offset = (
            self.master_measured_cp.M.Inverse() * desired_orientation
        )
        return desired_orientation

    def update_initial_state(self):
        self.master_cartesian_initial = self.master_measured_cp
        self.puppet_cartesian_initial = self.puppet_setpoint_cp
        self.update_align_offset()
        self.alignment_offset_initial = self.alignment_offset
        # TODO: missing base frame (?) here

    def set_scale(self, scale):
        self.scale = scale
        self.update_initial_state()

    def set_following(self, following):
        self.following = following
        self.following_master_body_servo_cf = None

    def lock_rotation(self, lock):
        self.rotation_locked = lock
        if not lock:
            self.set_following(False)
            self.set_desired_state(self.state.DISABLED)
        else:
            self.update_initial_state()
            if self.current_state == self.state.ENABLED and callable(
                getattr(self.master, "lock_orientation", None)
            ):
                self.master.lock_orientation(self.master_measured_cp.M)

    def lock_translation(self, lock):
        self.translation_locked = lock
        self.update_initial_state()

    def set_align_master(self, align_master):
        if callable(getattr(self.master, "lock_orientation", None)) and callable(
            getattr(self.master, "unlock_orientation", None)
        ):
            self.align_master = align_master
        else:
            if align_master:
                print(
                    f"{self.ral.node_name()}: unable to force master ({self.master.name()}) alignment, the device doesn't provide commands to lock/unlock orientation"
                )
            self.align_master = False
        if self.current_state == self.state.ENABLED:
            self.set_desired_state(self.state.DISABLED)

    def gripper_to_jaw(self, gripper_angle):
        return self.gripper_to_jaw_scale * gripper_angle + self.gripper_to_jaw_offset

    def jaw_to_gripper(self, jaw_angle):
        return (jaw_angle - self.gripper_to_jaw_offset) / self.gripper_to_jaw_scale

    def update_gripper_to_jaw_configuration(self):
        # jaw_min = self.gripper_zero
        # jaw_max = self.gripper_max

        self.gripper_to_jaw_position_min = self.jaw_min
        # TODO: add configuration_js? -- not really

        self.gripper_to_jaw_scale = self.jaw_max / (
            self.gripper_max - self.gripper_zero
        )
        self.gripper_to_jaw_offset = -self.gripper_zero / self.gripper_to_jaw_scale

    def __state_command_cb(self, msg):
        command = msg.string
        if command == "enable":
            self.set_desired_state(self.state.ENABLED)
        elif command == "disable":
            self.set_desired_state(self.state.DISABLED)
        elif command == "align_mtm":
            self.set_desired_state(self.state.ALIGNING_ARM)
        else:
            print(
                f"{self.ral.node_name()}: {command} doesn't seem to be a valid state_command"
            )

    def set_current_state(self, state):
        if state == self.state.DISABLED:
            print('Moving into state "DISABLED"')
        elif state == self.state.SETTING_ARMS_STATE:
            print('Moving into state "SETTING_ARMS"')
            self.enter_setting_arms_state()
        elif state == self.state.ALIGNING_ARM:
            print('Moving into state "ALIGNING_ARM"')
            self.enter_aligning_arm()
        elif state == self.state.ENABLED:
            print('Moving into state "ENABLED"')
            self.enter_enabled()
        else:
            raise RuntimeError("Invalid state")
        self.current_state = state

    def set_desired_state(self, state):
        self.desired_state = state
        self.operator_is_active = False

    def run(self):
        while not rospy.is_shutdown() and self.running:
            try:
                if self.current_state == self.state.DISABLED:
                    self.transition_disabled()
                elif self.current_state == self.state.SETTING_ARMS_STATE:
                    self.transition_setting_arms_state()
                elif self.current_state == self.state.ALIGNING_ARM:
                    self.transition_aligning_arm()
                elif self.current_state == self.state.ENABLED:
                    self.transition_enabled()
                self.run_all_states()
                if self.current_state != self.state.ENABLED:
                    self.puppet_virtual.hold()
                if self.current_state == self.state.ALIGNING_ARM:
                    self.run_aligning_arm()
                elif self.current_state == self.state.ENABLED:
                    self.run_enabled()
            except Exception as e:
                print(e)
                self.running = False


class mtm_teleop(object):
    class __ServoCf:
        def __init__(self, ral, expected_interval):
            self.__crtk_utils = crtk.utils(self, ral, expected_interval)
            self.__crtk_utils.add_servo_cf()

    class __Gripper:
        def __init__(self, ral, expected_interval):
            self.__crtk_utils = crtk.utils(self, ral, expected_interval)
            self.__crtk_utils.add_measured_js()

    def __init__(self, ral, arm_name, expected_interval=0.01):
        """Requires a arm name, this will be used to find the ROS
        topics for the arm being controlled.  For example if the
        user wants `PSM1`, the ROS topics will be from the namespace
        `PSM1`"""
        self.__name = arm_name
        self.__ral = ral.create_child(arm_name)

        # crtk features
        self.__crtk_utils = crtk.utils(self, self.__ral, expected_interval)

        self.__crtk_utils.add_operating_state()
        self.__crtk_utils.add_measured_cp()
        # self.__crtk_utils.add_measured_cv()
        self.__crtk_utils.add_setpoint_cp()
        self.__crtk_utils.add_move_cp()

        self.gripper = self.__Gripper(
            self.__ral.create_child("/gripper"), expected_interval
        )
        self.body = self.__ServoCf(self.__ral.create_child("body"), expected_interval)

        # publishers
        self.__lock_orientation_publisher = self.__ral.publisher(
            "lock_orientation", geometry_msgs.msg.Quaternion, latch=True, queue_size=1
        )
        self.__unlock_orientation_publisher = self.__ral.publisher(
            "unlock_orientation", std_msgs.msg.Empty, latch=True, queue_size=1
        )
        self.__use_gravity_compensation_pub = self.__ral.publisher(
            "/use_gravity_compensation", std_msgs.msg.Bool, latch=True, queue_size=1
        )

    def name(self):
        return self.__name

    def lock_orientation_as_is(self):
        "Lock orientation based on current orientation"
        current = self.setpoint_cp()
        self.lock_orientation(current.M)

    def lock_orientation(self, orientation):
        """Lock orientation, expects a PyKDL rotation matrix (PyKDL.Rotation)"""
        q = geometry_msgs.msg.Quaternion()
        q.x, q.y, q.z, q.w = orientation.GetQuaternion()
        self.__lock_orientation_publisher.publish(q)

    def unlock_orientation(self):
        "Unlock orientation"
        e = std_msgs.msg.Empty()
        self.__unlock_orientation_publisher.publish(e)

    def use_gravity_compensation(self, gravity_compensation):
        """Turn on/off gravity compensation in cartesian effort mode"""
        g = std_msgs.msg.Bool()
        g.data = gravity_compensation
        self.__use_gravity_compensation_pub.publish(g)


class psm_teleop(object):
    class __Jaw:
        def __init__(self, ral, expected_interval):
            self.__crtk_utils = crtk.utils(self, ral, expected_interval)
            self.__crtk_utils.add_setpoint_js()
            self.__crtk_utils.add_servo_jp()

    def __init__(self, ral, arm_name, expected_interval=0.01):
        """Requires a arm name, this will be used to find the ROS
        topics for the arm being controlled.  For example if the
        user wants `PSM1`, the ROS topics will be from the namespace
        `PSM1`"""
        self.__name = arm_name
        self.__ral = ral.create_child(arm_name)

        # crtk features
        self.__crtk_utils = crtk.utils(self, self.__ral, expected_interval)

        self.__crtk_utils.add_operating_state()
        self.__crtk_utils.add_setpoint_cp()
        self.__crtk_utils.add_servo_cp()
        self.__crtk_utils.add_hold()

        self.jaw = self.__Jaw(self.__ral.create_child("/jaw"), expected_interval)

    def name(self):
        return self.__name


class psm_ambf(object):
    class __Jaw:
        def __init__(self, parent_class):
            self.parent_class = parent_class

        def servo_jp(self, jp):
            self.parent_class.command_jp[6:8] = [jp[0] / 2, jp[0] / 2]
            self.parent_class.servo_jp(self.parent_class.command_jp)

        def setpoint_js(self, age=None):
            return np.array(
                [self.parent_class.measured_jp[6] + self.parent_class.measured_jp[7]]
            )

    def __init__(self, ral, arm_name, expected_interval=0.01):
        self.__name = arm_name
        self.__ral = ral.create_child(arm_name)

        # crtk features
        self.__crtk_utils = crtk.utils(self, self.__ral, expected_interval)

        self.__crtk_utils.add_measured_js()
        self.__crtk_utils.add_servo_jp()

        self.jaw = self.__Jaw(self)
        self.command_jp = np.zeros((8,))
        self.measured_jp = np.zeros((8,))

    def name(self):
        return self.__name

    def setpoint_cp(self, age=None):
        js = self.measured_js(age=age)
        self.measured_jp = js[0]
        cp = psm_solver.compute_FK(self.measured_jp[0:6], 6)
        return PyKDL.Frame(
            PyKDL.Rotation(
                cp[0, 0],
                cp[0, 1],
                cp[0, 2],
                cp[1, 0],
                cp[1, 1],
                cp[1, 2],
                cp[2, 0],
                cp[2, 1],
                cp[2, 2],
            ),
            PyKDL.Vector(cp[0, 3], cp[1, 3], cp[2, 3]),
        )

    def servo_cp(self, cp):
        jp = psm_solver.compute_IK(cp)
        self.command_jp[0:6] = jp[0:6]
        self.command_jp[2] = self.command_jp[2]
        self.servo_jp(self.command_jp)

    def hold(self):
        self.servo_jp(self.command_jp)


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
    args = parser.parse_args(argv)

    ral = crtk.ral("dvrk_python_teleoperation")
    mtm = mtm_teleop(ral, args.mtm, 4 * args.interval)
    psm = psm_teleop(ral, args.psm, 4 * args.interval)
    psm_virtual = psm_ambf(ral, "/ambf/env/psm2", 2 * args.interval)
    application = dvrk_teleoperation_ambf(
        ral,
        mtm,
        psm,
        psm_virtual,
        args.clutch,
        args.interval,
        operator_present_topic=args.operator,
        config_file_name="",
    )
    ral.spin_and_execute(application.run)
