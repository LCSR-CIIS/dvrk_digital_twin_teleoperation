import rospy
from std_msgs.msg import Bool
from numpy import random
import time

def com_loss_publisher():
    pub = rospy.Publisher('communication_loss', Bool, queue_size=1)
    pub_ar = rospy.Publisher('ar_activate', Bool, queue_size=1)
    rospy.init_node('communication_loss_talker', anonymous=True)

    i = 0
    comm_loss = False

    comm_good_mean = 1.2
    comm_good_std = 0.15
    comm_loss_mean = 0.6
    comm_loss_std = 0.1

    hz = 200
    rate = rospy.Rate(hz) # 200hz

    comm_good_period = random.normal(comm_good_mean, comm_good_std)
    comm_loss_period = random.normal(comm_loss_mean, comm_loss_std)

    print("TIME_period: " + str(comm_good_period) + " sec")
    print("LOSS_period: "  + str(comm_loss_period) + " sec")

    pub.publish(comm_loss)

    while not rospy.is_shutdown():
        i += 1

        if ((not comm_loss) and i == int(comm_good_period * hz)):
            comm_loss = True
            pub.publish(comm_loss)
            pub_ar.publish(comm_loss)
            i = 0

        if (comm_loss and i == int(comm_loss_period * hz)):
            comm_loss = False
            pub.publish(comm_loss)
            pub_ar.publish(comm_loss)
            i = 0

            comm_good_period = random.normal(comm_good_mean, comm_good_std)
            comm_loss_period = random.normal(comm_loss_mean, comm_loss_std)

            print("TIME_period: " + str(comm_good_period) + " sec")
            print("LOSS_period: "  + str(comm_loss_period) + " sec")

        rate.sleep()


if __name__ == '__main__':
    print("COMMUNICATION LOSS GENERATOR")
    # print("Please enter your seed:")
    # time.sleep(2)
    # num_seed = input(":\n")
    
    # print(f'Seed Number: {num_seed}')

    # random.seed(num_seed)
    try:
        com_loss_publisher()
    except rospy.ROSInterruptException:
        pass
