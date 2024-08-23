from dataclasses import dataclass, field
from typing import List
from ambf_client import Client
import time
import tf_conversions.posemath as pm
import PyKDL
import numpy as np
import rospy
from sensor_msgs.msg import JointState

initial_jp = [ -0.294, -0.0977, 0.154, 0.554, 0.00150, 0.0061 ]

@dataclass
class PsmSub:
    joint_state: np.ndarray = field(init=False, default_factory=list)

    def __post_init__(self):
        self.joint_state = np.array(initial_jp)
        self.markers_sub = rospy.Subscriber(
            "/PSM2/measured_js/", JointState, self._callback
        )

    def _callback(self, msg: JointState):
        self.joint_state = np.array(msg.position)

    def get_js(self):
        return self.joint_state.copy()


def setup():

    _client = Client()
    _client.connect()
    time.sleep(0.2)
    psm = _client.get_obj_handle("/ambf/env/psm2/base")
    camera = _client.get_obj_handle("main_camera")
    time.sleep(0.2)

    # Set initial cam pose base on hand-eye calibration
    set_camera_pose(camera)

    return _client, psm, camera


def set_camera_pose( camera_handle):
    # fmt: off
    # from hand-eye calibration
    cam_opencv_T_base = [[-0.9367793693845268, 0.04436124854636563, -0.3470972381378686, -0.03999301195527639],
                        [0.32436582916733303, 0.4821963365586197, -0.8138018812205062, -0.10651497624643151],
                        [0.13126774913937228, -0.8749392965439683, -0.4661008532485529, 0.1596443367059988],
                        [0.0, 0.0, 0.0, 1.0] ]
    # fmt: on
    cam_opencv_T_base = np.array(cam_opencv_T_base)
    base_T_cam_opencv = np.linalg.inv(cam_opencv_T_base) 

    cam_opencv_T_cam_ambf = np.array(
        [[0, 1, 0, 0], [0, 0, -1, 0], [-1, 0, 0, 0], [0, 0, 0, 1]]
    )
    base_T_cam_ambf = base_T_cam_opencv @ cam_opencv_T_cam_ambf
    base_T_cam_ambf = pm.toMsg(pm.fromMatrix(base_T_cam_ambf))

    camera_handle.set_pose(base_T_cam_ambf)

def virtual_psm_measured_jp(psm_base, joint_state):
    for i in range(6):
        psm_base.set_joint_pos(i, joint_state[i])

def main():
    _client, virtual_psm, virtual_camera = setup()

    psm_sub = PsmSub()

    while not rospy.is_shutdown():
        joint_state = psm_sub.get_js()
        virtual_psm_measured_jp(virtual_psm, joint_state)

        time.sleep(0.033)  # 30 Hz

    print("finishing program ...")



if __name__ == "__main__":
    main()
