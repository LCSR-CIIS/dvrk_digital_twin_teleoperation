import dvrk
import numpy as np
import yaml
import os
import crtk
import rospy


class PSMdVRK:
    # simplified jaw class to close gripper
    class __jaw_device:
        def __init__(self, ral, expected_interval, operating_state_instance):
            self.__crtk_utils = crtk.utils(
                self, ral, expected_interval, operating_state_instance
            )
            self.__crtk_utils.add_move_jp()
            self.__crtk_utils.add_servo_jp()

    def __init__(self, ral, arm_name, ros_namespace="", expected_interval=0.01):
        # ROS initialization
        if not rospy.get_node_uri():
            rospy.init_node(
                "simplified_psm_class", anonymous=False, log_level=rospy.WARN
            )
        # populate this class with all the ROS topics we need
        self.__ral = ral.create_child(arm_name)
        self.crtk_utils = crtk.utils(self, self.__ral, expected_interval)
        self.crtk_utils.add_operating_state()
        self.crtk_utils.add_servo_jp()
        self.crtk_utils.add_move_jp()
        self.crtk_utils.add_servo_cp()
        self.crtk_utils.add_move_cp()
        self.crtk_utils.add_measured_js()
        self.crtk_utils.add_measured_cp()
        jaw_ral = self.ral().create_child("/jaw")
        self.jaw = self.__jaw_device(
            jaw_ral, expected_interval, operating_state_instance=self
        )
        self.namespace = ros_namespace
        self.name = arm_name

    def ral(self):
        return self.__ral

    def check_connections(self, timeout=5.0):
        self.__ral.check_connections(timeout)


ral = crtk.ral("Teleop_Loss")
psm_handle = PSMdVRK(ral, arm_name="PSM2", expected_interval=0.01)
ral.check_connections()

state = yaml.load(open(os.path.join(os.path.dirname(__file__), "PSM2_state.yaml"), "r"))
jaw_state = yaml.load(
    open(os.path.join(os.path.dirname(__file__), "JAW2_state.yaml"), "r")
)
position = state["position"]
jaw_position = jaw_state["position"]
print(position, jaw_position)
pos_np = np.array(position)
jaw_pos_np = np.array(jaw_position)
# move ecm to a position
psm_handle.move_jp(pos_np).wait()
psm_handle.jaw.move_jp(jaw_pos_np).wait()  # set jaw to 0
# psm_handle.move_jp(np.array([-0.3,0.6,0.15,0.6,0.6])).wait()
# psm_handle.jaw.move_jp(np.array([0.2])).wait()

print("PSM is at the desired position")

# Move_jp
