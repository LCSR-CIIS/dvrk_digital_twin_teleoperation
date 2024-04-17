import argparse
import crtk
import cv2
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import numpy as np
import sensor_msgs.msg
import std_msgs.msg
import sys

class image_blender:
    def __init__(self, ral, source_topic, overlay_topic, result_topic):
        self.ral = ral
        self.alpha = 0.5
        self.bridge = CvBridge()

        self.source = np.array([0])
        self.overlay = np.array([0])
        self.result = np.array([0])
        self.result_img = sensor_msgs.msg.Image()
        self.update = False

        self._fig = plt.figure()
        self._ax = self._fig.add_subplot(111)
        
        self.result_pub = self.ral.publisher(result_topic,
                                             sensor_msgs.msg.Image)

        self.__source_sub = self.ral.subscriber(source_topic,
                                                sensor_msgs.msg.Image,
                                                self.__source_cb)
        
        self.__overlay_sub = self.ral.subscriber(overlay_topic,
                                                sensor_msgs.msg.Image,
                                                self.__overlay_cb)
        
        self.__comm_loss_sub = self.ral.subscriber('/communication_loss',
                                                   std_msgs.msg.Bool,
                                                   self.__comm_loss_cb,
                                                   latch = True)
        
    def __source_cb(self, msg):
        msg.encoding = "bgr8"
        self.source = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'rgb8')
        self.update = True
        
    def __overlay_cb(self, msg):
        msg.encoding = "bgr8"
        self.overlay = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'rgb8')
        self.update = True

    def __comm_loss_cb(self, value):
        if value != None:
            self.comm_loss = value.data
        self.handle_comm_loss(self.comm_loss)

    def handle_comm_loss(self, loss):
        if loss:
            self.last_source_before_loss = self.source

    def run(self):
        while not self.ral.is_shutdown():
            if self.update:
                if not self.comm_loss:
                    if len(self.source.shape) > 1:
                        self.result = self.source
                else:
                    if len(self.last_source_before_loss.shape) > 1 and len(self.overlay.shape) > 1:
                        self.result = np.uint8(self.alpha*self.last_source_before_loss + (1-self.alpha)*cv2.resize(self.overlay, (self.last_source_before_loss.shape[1], self.last_source_before_loss.shape[0]), interpolation=cv2.INTER_NEAREST))
                self.update = False
            if len(self.result.shape) > 1:
                cv2.imshow('Left', self.result)
                cv2.waitKey(1)
                cv2.imshow('Right', self.result)
                cv2.waitKey(1)


if __name__ == '__main__':
    # extract ros arguments (e.g. __ns:= for namespace)
    argv = crtk.ral.parse_argv(sys.argv[1:]) # skip argv[0], script name

    # parse arguments
    parser = argparse.ArgumentParser(description = __doc__,
                                     formatter_class = argparse.RawTextHelpFormatter)
    parser.add_argument('-s', '--source', type = str, default='/jhu_daVinci/decklink/left/image_rect_color',
                        help = 'ROS topic for source image')
    parser.add_argument('-o', '--overlay', type = str, default='/ambf/env/cameras/cameraL/ImageData',
                        help = 'ROS topic for overlay image')
    parser.add_argument('-r', '--result', type = str, default='/blending_result',
                        help = 'ROS topic for result image')
    args = parser.parse_args(argv)

    ral = crtk.ral('image_blender')
    application = image_blender(ral, args.source, args.overlay, args.result)
    ral.spin_and_execute(application.run)