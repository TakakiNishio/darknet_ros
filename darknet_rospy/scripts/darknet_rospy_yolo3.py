#!/usr/bin/env python
import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# import sys
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

from ctypes import *
import math
import random
import cv2
import colorsys
import numpy as np
import os
import shutil
import argparse

from darknet import *


class DarknetPyNode:

    def __init__(self):

        rospy.init_node('darknet_ros', anonymous=True)

        rospack = rospkg.RosPack()
        file_path = rospack.get_path('darknet_rospy')
        net_file = file_path+"/../darknet/cfg/yolov3.cfg"
        weights_file = file_path+"/../darknet/yolov3.weights"
        meta_file = file_path+"/../darknet/cfg/coco.data"
        names_file = file_path+"/../darknet/data/coco.names"

        if os.path.isdir('data'):
            shutil.rmtree('data')

        os.mkdir("data")
        shutil.copy(names_file, "data")
        self.net = load_net(net_file.encode('utf-8'), weights_file.encode('utf-8'), 0)
        self.meta = load_meta(meta_file.encode('utf-8'))
        shutil.rmtree("data")

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_color", Image, self.image_callback, queue_size=1, buff_size=2**24)
        self.result_image_pub = rospy.Publisher('recognition_result', Image, queue_size=1)
        self.color = (0,255,0)
        rospy.loginfo("Initialized DarknetPyNode")

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.darknet_detection(cv_image)
        except CvBridgeError as e:
            print(e)

    def darknet_detection(self, cv_image):
        r = detect_np(self.net, self.meta, cv_image)
        for i in r:
            x, y, w, h = i[2][0], i[2][1], i[2][2], i[2][3]
            xmin, ymin, xmax, ymax = convertBack(float(x), float(y), float(w), float(h))
            pt1 = (xmin, ymin)
            pt2 = (xmax, ymax)
            cv2.rectangle(cv_image, pt1, pt2, self.color, 2)
            cv2.putText(cv_image, i[0].decode() + " [" + str(round(i[1] * 100, 2)) + "%]",
                        (pt1[0]+2, pt1[1]+25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, self.color, 3)

        self.result_image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))


if __name__ == "__main__":

    DarknetPyNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
         print("Shutting down")
