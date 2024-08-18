from ambf_client import Client
import time
import tf_conversions.posemath as pm
import PyKDL
import numpy as np

##################
# AMBF client setup

_client = Client()

# Connect the client which in turn creates callable objects from ROS topics
# and initiates a shared pool of threads for bidrectional communication
_client.connect()
time.sleep(0.2)  # Sleep after connecting the client

psm_gripper = _client.get_obj_handle("Cube")
psm_base = _client.get_obj_handle("/ambf/env/psm2/base")
psm_gripper = _client.get_obj_handle("/ambf/env/psm2/toolyawlink")

time.sleep(0.2)  # Sleep after getting the object handles

print(_client.get_obj_names())
print(psm_gripper.get_pos())

##################
# Set robot joints

num_joints = psm_base.get_num_joints()  # Get the number of joints of this object
children_names = (
    psm_base.get_children_names()
)  # Get a list of children names belonging to this obj

# fmt: off
joint = [ -0.908286058803293, -0.2719792063644388, 0.10170302453, 1.595530779973746, 0.011067556481000676, 0.485948315052397 ]
# fmt: on
print(num_joints)
print(children_names)

for i in range(6):
    psm_base.set_joint_pos(i, joint[i])

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


input("Press Enter to finish...")  # Wait for user input before closing the client
