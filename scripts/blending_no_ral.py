import argparse
import cv2
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import numpy as np
import sensor_msgs.msg
import sys
import rospy

class image_blender:
    def __init__(self, source_topic, overlay_topic, result_topic):
        self.alpha = 0.7
        self.bridge = CvBridge()

        self.source = np.array([0])
        self.overlay = np.array([0])
        self.result = np.array([0])
        self.result_img = sensor_msgs.msg.Image()
        self.update = False

        self._fig = plt.figure()
        self._ax = self._fig.add_subplot(111)
        
        self.result_pub = rospy.Publisher(result_topic,
                                             sensor_msgs.msg.Image,queue_size=10)

        self.__source_sub = rospy.Subscriber(source_topic,
                                                sensor_msgs.msg.Image,
                                                self.__source_cb)
        
        self.__overlay_sub = rospy.Subscriber(overlay_topic,
                                                sensor_msgs.msg.Image,
                                                self.__overlay_cb)
        
    def __source_cb(self, msg):
        # msg.encoding = "bgr8" # Why hardcode the encoding?
        self.source = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'rgb8')
        self.update = True
        
    def __overlay_cb(self, msg):
        msg.encoding = "bgr8"
        self.overlay = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'rgb8')
        self.update = True

    def run(self):
        while not rospy.is_shutdown():
            if self.update:
                if len(self.source.shape) > 1 and len(self.overlay.shape) > 1:
                    self.result = np.uint8(self.alpha*self.source + (1-self.alpha)*cv2.resize(self.overlay, (self.source.shape[1], self.source.shape[0]), interpolation=cv2.INTER_NEAREST))
            self.update = False
            if len(self.result.shape) > 1:
                cv2.imshow('Result', self.result)
                cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node('image_blender', anonymous=True)
    # extract ros arguments (e.g. __ns:= for namespace)

    # parse arguments
    parser = argparse.ArgumentParser(description = __doc__,
                                     formatter_class = argparse.RawTextHelpFormatter)
    parser.add_argument('-s', '--source', type = str, default='/zedm/zed_node/left/image_rect_color',
                        help = 'ROS topic for source image')
    parser.add_argument('-o', '--overlay', type = str, default='/ambf/env/cameras/zed_m_camera/ImageData',
                        help = 'ROS topic for overlay image')
    parser.add_argument('-r', '--result', type = str, default='/blending_result',
                        help = 'ROS topic for result image')
    args = parser.parse_args()

    application = image_blender(args.source, args.overlay, args.result)

    application.run() 

    print("Shutting down")