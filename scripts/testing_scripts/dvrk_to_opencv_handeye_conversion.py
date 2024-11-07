#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import tf_conversions.posemath as pm
import numpy as np 

def pose_to_matrix(pose):
    # Convert the pose to a transformation matrix
    pose_tf = pm.fromMsg(pose)
    matrix = pm.toMatrix(pose_tf)
    return matrix

def convert_to_opencv_convention(matrix):
    to_dvrk = np.eye(4)
    to_dvrk[0,0] = -to_dvrk[0,0]
    to_dvrk[1,1] = -to_dvrk[1,1]

    return to_dvrk @ matrix

def pose_listener():
    rospy.init_node('pose_matrix_listener', anonymous=True)

    # Wait until a PoseStamped message is received from the specified topic
    rospy.loginfo("Waiting for a PoseStamped message...")
    msg = rospy.wait_for_message("/SUJ/PSM2/measured_cp", PoseStamped)
    rospy.loginfo("PoseStamped message received")

    # Extract the pose part from PoseStamped
    pose = msg.pose

    # Convert the pose to a transformation matrix
    matrix_dvrk = pose_to_matrix(pose)
    matrix_opencv = convert_to_opencv_convention(matrix_dvrk)

    matrix_dvrk_str = np.array2string(np.array(matrix_dvrk), separator=',', suppress_small=True, formatter={'float_kind': lambda x: "%.6f" % x})
    matrix_opencv_str = np.array2string(np.array(matrix_opencv), separator=',', suppress_small=True, formatter={'float_kind': lambda x: "%.6f" % x})

    # Print the matrix
    rospy.loginfo("Transformation Matrix (DVRK convention):\n%s", matrix_dvrk_str)
    rospy.loginfo("Transformation Matrix (Opencv convention):\n%s", matrix_opencv_str)

if __name__ == '__main__':
    try:
        pose_listener()
    except rospy.ROSInterruptException:
        pass