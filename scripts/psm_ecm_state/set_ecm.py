import dvrk
import numpy as np
import yaml
import os
import crtk
import rospy
import click
from pathlib import Path

class ECMdVRK:
    # simplified jaw class to close gripper
    # class __jaw_device:
    #     def __init__(self, ral, expected_interval, operating_state_instance):
    #         self.__crtk_utils = crtk.utils(self, ral, expected_interval, operating_state_instance)
    #         self.__crtk_utils.add_move_jp()
    #         self.__crtk_utils.add_servo_jp()

    def __init__(self, ral, arm_name, ros_namespace="", expected_interval=0.01):
        # ROS initialization
        if not rospy.get_node_uri():
            rospy.init_node(
                "simplified_ecm_class", anonymous=False, log_level=rospy.WARN
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
        # jaw_ral = self.ral().create_child('/jaw')
        # self.jaw = self.__jaw_device(jaw_ral, expected_interval, operating_state_instance=self)
        self.namespace = ros_namespace
        self.name = arm_name

    def ral(self):
        return self.__ral

    def check_connections(self, timeout=5.0):
        self.__ral.check_connections(timeout)


@click.command()
@click.option("--path", required=True, help="Path to ecm state yaml file")
def main(path: Path):
    path = Path(path)
    path = path.resolve()

    ral = crtk.ral("Teleop_Loss")
    ecm_handle = ECMdVRK(ral, arm_name="ECM", expected_interval=0.01)
    ral.check_connections()

    state = yaml.safe_load(open(str(path), "r"))
    position = state["position"]
    print(position)
    pos_np = np.array(position)
    # move ecm to a position

    ecm_handle.move_jp(pos_np).wait()
    # Move_jp

if __name__ == "__main__":
    main()
