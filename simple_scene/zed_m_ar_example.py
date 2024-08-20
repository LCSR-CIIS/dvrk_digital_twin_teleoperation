from dataclasses import dataclass, field
from typing import List
from ambf_client import Client
import time
import tf_conversions.posemath as pm
import PyKDL
import numpy as np
import rospy
from aruco_detection.msg import MarkerPose, MarkerPoseArray


@dataclass
class ArucoMarkerSubscriber:
    marker_arr: List[MarkerPose] = field(init=False, default_factory=list)

    def __post_init__(self):
        self.markers_sub = rospy.Subscriber(
            "/aruco/marker_poses", MarkerPoseArray, self._callback
        )

    def __len__(self):
        return len(self.marker_arr)

    def _callback(self, msg: MarkerPoseArray):
        self.marker_arr: List[MarkerPose] = msg.markers

    def create_copy_of_markers_arr(self):
        return self.marker_arr.copy()


def setup():

    _client = Client()
    _client.connect()
    time.sleep(0.2)
    cube = _client.get_obj_handle("Cube")
    camera = _client.get_obj_handle("main_camera")
    time.sleep(0.2)

    return _client, cube, camera


def set_camera_pose(marker_T_cam_cv: np.ndarray, camera_handle):

    # # fmt: off
    # # from hand-eye calibration
    # base_T_cam_opencv = [[ -0.9606688751203619, 0.18354521570036922, -0.20839017771594118, -0.10692537987763193, ],
    #                     [ 0.21869689068239725, 0.9625158981393869, -0.16042074627298478, -0.008656956380724598, ],
    #                     [ 0.17113439859019633, -0.19968550178326597, -0.9647998331243302, 0.1386334372566933, ],
    #                     [0.0, 0.0, 0.0, 1.0] ]
    # # fmt: on
    # base_T_cam_opencv = np.array(base_T_cam_opencv)

    cam_cv_T_cam_ambf = np.array(
        [[0, 1, 0, 0], [0, 0, -1, 0], [-1, 0, 0, 0], [0, 0, 0, 1]]
    )
    marker_T_cam_ambf = marker_T_cam_cv @ cam_cv_T_cam_ambf
    marker_T_cam_ambf = pm.toMsg(pm.fromMatrix(marker_T_cam_ambf))

    camera_handle.set_pose(marker_T_cam_ambf)


def get_marker_pose(aruco_marker_sub: ArucoMarkerSubscriber):
    markers = aruco_marker_sub.create_copy_of_markers_arr()
    for m in markers:
        if m.id == 0:
            marker_pose = pm.toMatrix(pm.fromMsg(m.pose))
            return marker_pose
    
    return None


def main():
    _client, cube, camera = setup()
    aruco_marker_sub = ArucoMarkerSubscriber()

    while not rospy.is_shutdown():
        cam_cv_T_marker = get_marker_pose(aruco_marker_sub)
        if cam_cv_T_marker is not None:
            marker_T_cam_cv = np.linalg.inv(cam_cv_T_marker)
            set_camera_pose(marker_T_cam_cv, camera)

        time.sleep(0.033)  # 30 Hz

    print("finishing program ...")

    markers = aruco_marker_sub.create_copy_of_markers_arr()
    for m in markers:
        print(m.id)
        print(m.pose)

        if m.id == 0:
            print(type(m.pose))
            cam_pose = pm.toMatrix(pm.fromMsg(m.pose))
            print(cam_pose)


if __name__ == "__main__":
    main()
