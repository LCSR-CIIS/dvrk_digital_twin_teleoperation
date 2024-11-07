import rospy
from std_msgs.msg import Bool
import time
import threading

def com_loss_publisher():
    pub = rospy.Publisher('communication_loss', Bool, queue_size=1)
    pub_visibility = rospy.Publisher('ar_activate', Bool, queue_size=1)
    rospy.init_node('communication_loss_talker', anonymous=True)

    flag = False  # Initial state of communication (no loss)
    hz = 200
    rate = rospy.Rate(hz)  # 200hz
    pub_visibility.publish(flag)
    print("Press 'enter' to toggle communication loss.")

    def key_listener():
        nonlocal flag
        while not rospy.is_shutdown():
            key = input().strip().lower()
            #if key is space then toggle the flag
            if key == '':
                flag = not flag
                if flag:
                    print("Communication Loss Triggered.")
                else:
                    print("Communication Recovered.")
            if key == 'l':
                flag = True
                pub.publish(flag)
                print("Communication Loss Triggered.")
            elif key == 'r':
                flag = False
                pub.publish(flag)
                print("Communication Recovered.")
            elif key == 'q':
                break

    # Run the key listener in a separate thread to avoid blocking the main loop
    key_thread = threading.Thread(target=key_listener)
    key_thread.daemon = True
    key_thread.start()

    # Main loop just to maintain the ROS node activity
    while not rospy.is_shutdown():
        # Publish the current flag status periodically
        pub.publish(flag)
        pub_visibility.publish(flag)
        rate.sleep()
        

com_loss_publisher()