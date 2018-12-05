#!/usr/bin/env python
import rospy
import rospkg

import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

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


if __name__ == "__main__":

    cap = cv2.VideoCapture(0)
    ret, img = cap.read()

    rospack = rospkg.RosPack()
    file_path = rospack.get_path('darknet_ros')

    net_filename = file_path+"/../darknet/cfg/yolov3.cfg"
    weights_filename = file_path+"/../darknet/yolov3.weights"
    meta_filename = file_path+"/../darknet/cfg/coco.data"

    os.mkdir("data")
    shutil.copy(file_path+"/../darknet/data/coco.names","data")

    net = load_net(net_filename.encode('utf-8'), weights_filename.encode('utf-8'), 0)
    meta = load_meta(meta_filename.encode('utf-8'))

    shutil.rmtree("data")

    color = (0,255,0)

    cv2.namedWindow("result", cv2.WINDOW_NORMAL)

    while(1):
        ret, img = cap.read()
        r = detect_np(net, meta, img)
        for i in r:
            x, y, w, h = i[2][0], i[2][1], i[2][2], i[2][3]
            xmin, ymin, xmax, ymax = convertBack(float(x), float(y), float(w), float(h))
            pt1 = (xmin, ymin)
            pt2 = (xmax, ymax)
            cv2.rectangle(img, pt1, pt2, color, 2)
            cv2.putText(img, i[0].decode() + " [" + str(round(i[1] * 100, 2)) + "%]",
                        (pt1[0]+2, pt1[1]+25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 3)
        cv2.imshow("result", img)
        k = cv2.waitKey(1) & 0xFF

        if k == 27:
            cv2.destroyAllWindows()
            exit()
