import argparse
from dataclasses import dataclass, field
import json
import pathlib
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


def setup(cam_opencv_T_base: np.ndarray):

    _client = Client()
    _client.connect()
    time.sleep(0.2)
    psm = _client.get_obj_handle("/ambf/env/psm2/base")
    camera = _client.get_obj_handle("main_camera")
    time.sleep(0.2)

    # Set initial cam pose base on hand-eye calibration
    set_camera_pose(camera, cam_opencv_T_base)

    return _client, psm, camera


def set_camera_pose(camera_handle, cam_opencv_T_base: np.ndarray):
    # # fmt: off
    # cam_opencv_T_base = [[-0.8790781792313629, 0.3412618733306592, -0.3328090873310377, -0.08238268273316497], 
    #                      [0.33239054126068474, 0.9392881778897073, 0.08517186716905183, -0.007596158536450624], 
    #                      [0.34166955216948763, -0.03574986276173252, -0.9391399599808413, 0.09819970244543609], 
    #                      [0.0, 0.0, 0.0, 1.0]]
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

def load_hand_eye_calibration(json_file: pathlib.Path) -> np.ndarray:

    if not json_file.exists():
        raise FileNotFoundError(f"Hand-eye calibration file not found: {json_file}")

    with open(json_file, "r") as f:
        data = json.load(f)

    cam_T_robot_base = np.array(data['base-frame']['transform']).reshape(4, 4)

    return cam_T_robot_base

def main():

    parser = argparse.ArgumentParser()
    # fmt: off
    parser.add_argument(
        "-H", "--hand-eye-json", type=str, required=True,
        help="hand-eye calibration matrix in JSON format using OpenCV coordinate system",
    )
    # fmt: on
    args = parser.parse_args()

    hand_eye_path = pathlib.Path(args.hand_eye_json)
    cam_opencv_T_base = load_hand_eye_calibration(hand_eye_path)

    _client, virtual_psm, virtual_camera = setup(cam_opencv_T_base)

    psm_sub = PsmSub()

    while not rospy.is_shutdown():
        joint_state = psm_sub.get_js()
        virtual_psm_measured_jp(virtual_psm, joint_state)

        time.sleep(0.033)  # 30 Hz

    print("finishing program ...")



if __name__ == "__main__":
    main()
