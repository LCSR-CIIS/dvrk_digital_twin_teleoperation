import rospy
from std_msgs.msg import Bool
from numpy import random
import time
import click


def com_loss_publisher(ar_overlay):
    pub = rospy.Publisher("communication_loss", Bool, queue_size=1)
    pub_ar = rospy.Publisher("ar_activate", Bool, queue_size=1)
    rospy.init_node("communication_loss_talker", anonymous=True)

    i = 0
    comm_loss = False

    comm_good_mean = 3.2
    comm_good_std = 0.15
    comm_loss_mean = 0.8
    comm_loss_std = 0.1

    hz = 50
    rate = rospy.Rate(hz)  # 200hz

    comm_good_period = random.normal(comm_good_mean, comm_good_std)
    comm_loss_period = random.normal(comm_loss_mean, comm_loss_std)

    print("TIME_period: " + str(comm_good_period) + " sec")
    print("LOSS_period: " + str(comm_loss_period) + " sec")

    pub.publish(comm_loss)

    while not rospy.is_shutdown():
        i += 1

        if (not comm_loss) and i == int(comm_good_period * hz):
            comm_loss = True
            pub.publish(comm_loss)
            if ar_overlay:
                pub_ar.publish(comm_loss)
            i = 0

        if comm_loss and i == int(comm_loss_period * hz):
            comm_loss = False
            pub.publish(comm_loss)
            if ar_overlay:
                pub_ar.publish(comm_loss)
            i = 0

            comm_good_period = random.normal(comm_good_mean, comm_good_std)
            comm_loss_period = random.normal(comm_loss_mean, comm_loss_std)

            print("TIME_period: " + str(comm_good_period) + " sec")
            print("LOSS_period: " + str(comm_loss_period) + " sec")

        rate.sleep()


@click.command()
@click.option(
    "--ar_overlay", "-a", default=False, help="Toggle AR overlay on/off with comm loss", type=bool
)
def main(ar_overlay):
    print("COMMUNICATION LOSS GENERATOR")
    # print("Please enter your seed:")
    # time.sleep(2)
    # num_seed = input(":\n")

    # print(f'Seed Number: {num_seed}')

    # random.seed(num_seed)
    try:
        com_loss_publisher(ar_overlay)
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
