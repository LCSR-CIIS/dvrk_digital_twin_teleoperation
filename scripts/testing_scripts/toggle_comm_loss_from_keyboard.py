import rospy
from std_msgs.msg import Bool
import random
import time

def com_loss_publisher():
    pub = rospy.Publisher('communication_loss', Bool, queue_size=1)
    rospy.init_node('communication_loss_talker', anonymous=True)

    comm_loss_status = False

    pub.publish(comm_loss_status)
    print(f"press enter to toogle communication loss")
    print(f"communication loss status: {comm_loss_status}")
    while not rospy.is_shutdown():
        input("")
        comm_loss_status = not comm_loss_status
        pub.publish(comm_loss_status)
        print(f"communication loss status: {comm_loss_status}")



if __name__ == '__main__':
    print("Generate Communication loss with keyboard")

    try:
        com_loss_publisher()
    except rospy.ROSInterruptException:
        pass
