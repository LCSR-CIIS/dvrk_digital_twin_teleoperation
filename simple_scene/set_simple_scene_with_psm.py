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
camera = _client.get_obj_handle("main_camera")

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
joint = [ -0.2943210725695703, -0.09773890725618539, 0.15416647341, 0.5548395904600515, 0.001503001497419845, 0.006125489671989966 ]
# fmt: on
print(num_joints)
print(children_names)

for i in range(6):
    psm_base.set_joint_pos(i, joint[i])

#################
# Set camera pose

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

camera.set_pose(base_T_cam_ambf)

time.sleep(1.0)


input("Press Enter to finish...")  # Wait for user input before closing the client
