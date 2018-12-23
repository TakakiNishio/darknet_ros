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
import copy
# import argparse

from darknet import *
from darknet_rospy_msgs.msg import DarkBox
from darknet_rospy_msgs.msg import DarkBoxArray


class DarknetPyNode:

    def __init__(self):

        rospy.init_node('darknet_ros', anonymous=True)

        rospack = rospkg.RosPack()
        file_path = rospack.get_path('darknet_rospy')
        net_file = file_path+rospy.get_param('~net_file', "/../darknet/cfg/yolov3.cfg")
        weights_file = file_path+rospy.get_param('~weights_file', "/../darknet/yolov3.weights")
        meta_file = file_path+rospy.get_param('~meta_file', "/../darknet/cfg/coco.data")
        names_file = file_path+rospy.get_param('~names_file', "/../darknet/data/coco.names")
        input_image_topic = rospy.get_param('~input_image_topic', "/camera/image_color")
        output_image_topic = rospy.get_param('~output_image_topic', "/darknet/recognition_result_image")
        output_topic = rospy.get_param('~output_topic', "/darknet/recognition_result")
        cropping_area = rospy.get_param('~cropping_area', None)

        if cropping_area is not None:
            cropping_area = [[int(x) for x in ss.lstrip('[').split(' ')][0] for ss in cropping_area.rstrip(']').split(',')]
        self.cropping_area = cropping_area

        if os.path.isdir('data'):
            shutil.rmtree('data')

        os.mkdir("data")
        shutil.copy(names_file, "data")
        self.net = load_net(net_file.encode('utf-8'), weights_file.encode('utf-8'), 0)
        self.meta = load_meta(meta_file.encode('utf-8'))
        shutil.rmtree("data")

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(input_image_topic, Image, self.image_callback, queue_size=1, buff_size=2**24)
        self.result_image_pub = rospy.Publisher(output_image_topic, Image, queue_size=1)
        self.result_pub = rospy.Publisher(output_topic, DarkBoxArray, queue_size=1)

        self.color = (0,255,0)
        rospy.loginfo("Initialized DarknetPyNode")

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.darknet_detection(cv_image)
        except CvBridgeError as e:
            print(e)

    def darknet_detection(self, cv_image):

        # cv_image = cv2.resize(cv_image, None, fx=0.5, fy=0.5)

        darkbox_array = DarkBoxArray()
        darkbox_list = []

        cv_image_ = copy.deepcopy(cv_image)
        
        if self.cropping_area is not None:
            cv_image = cv_image[self.cropping_area[0]:self.cropping_area[1],
                                self.cropping_area[2]:self.cropping_area[3]]

        r = detect_np(self.net, self.meta, cv_image)
        for i in r:
            x, y, w, h = i[2][0], i[2][1], i[2][2], i[2][3]
            xmin, ymin, xmax, ymax = convertBack(float(x), float(y), float(w), float(h))
            probability = round(i[1], 4)
            label = i[0].decode()

            if self.cropping_area is not None:
                xmin += self.cropping_area[2]
                xmax += self.cropping_area[2]
                ymin += self.cropping_area[0]
                ymax += self.cropping_area[0]

            darkbox = DarkBox()
            darkbox.probability = probability
            darkbox.label = label
            darkbox.xmin = xmin
            darkbox.ymin = ymin
            darkbox.xmax = xmax
            darkbox.ymax = ymax

            pt1 = (xmin, ymin)
            pt2 = (xmax, ymax)
            cv2.rectangle(cv_image_, pt1, pt2, self.color, 2)
            cv2.putText(cv_image_, label + " [" + str(probability*100) + "%]",
                        (pt1[0]+2, pt1[1]+25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, self.color, 3)

            darkbox_list.append(darkbox)

        darkbox_array.dark_array = darkbox_list
        self.result_image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image_, "bgr8"))
        self.result_pub.publish(darkbox_array)


if __name__ == "__main__":

    DarknetPyNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
         print("Shutting down")
