import rospy
from std_msgs.msg import Bool
import random
import time

def com_loss_publisher():
    pub = rospy.Publisher('ar_activate', Bool, queue_size=1)
    rospy.init_node('ar_status_pub', anonymous=True)

    comm_loss_status = False

    pub.publish(comm_loss_status)
    print(f"press enter to toogle AR overlay")
    print(f"AR overlay active: {comm_loss_status}")
    while not rospy.is_shutdown():
        input("")
        comm_loss_status = not comm_loss_status
        pub.publish(comm_loss_status)
        print(f"AR overlay active: {comm_loss_status}")



if __name__ == '__main__':
    print("Toggle AR overlay with keyboard")

    try:
        com_loss_publisher()
    except rospy.ROSInterruptException:
        pass