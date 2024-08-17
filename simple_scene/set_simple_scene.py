from ambf_client import Client
import time
import tf_conversions.posemath as pm
import PyKDL
import numpy as np

# Create a instance of the client
_client = Client()

# Connect the client which in turn creates callable objects from ROS topics
# and initiates a shared pool of threads for bidrectional communication
_client.connect()
time.sleep(0.2)

psm_gripper = _client.get_obj_handle("Cube")

# You can print the names of objects found
print(_client.get_obj_names())
print(psm_gripper.get_pos())

##################
# Set gripper pose

gripper_pose = [-0.0723643347765138, 0.025577775250596457, -0.0563267293012299] # from PSM2/local/measured_cp
psm_gripper.set_pos(gripper_pose[0], gripper_pose[1], gripper_pose[2])

print(psm_gripper.get_pos())


#################
# Set camera pose

# fmt: off
# from hand-eye calibration
base_T_cam_opencv = [[ -0.9606688751203619, 0.18354521570036922, -0.20839017771594118, -0.10692537987763193, ],
                    [ 0.21869689068239725, 0.9625158981393869, -0.16042074627298478, -0.008656956380724598, ],
                    [ 0.17113439859019633, -0.19968550178326597, -0.9647998331243302, 0.1386334372566933, ],
                    [0.0, 0.0, 0.0, 1.0] ]
# fmt: on
base_T_cam_opencv = np.array(base_T_cam_opencv)
cam_opencv_T_cam_ambf = np.array(
    [[0, 1, 0, 0], [0, 0, -1, 0], [-1, 0, 0, 0], [0, 0, 0, 1]]
)
base_T_cam_ambf = base_T_cam_opencv @ cam_opencv_T_cam_ambf

base_T_cam_ambf = pm.toMsg(pm.fromMatrix(base_T_cam_ambf))
camera = _client.get_obj_handle("cameraL")
camera.set_pose(base_T_cam_ambf)

time.sleep(1.0)
print("psm_gripper")
print(psm_gripper.get_pos())
print("camera")
print(camera.get_pos())
