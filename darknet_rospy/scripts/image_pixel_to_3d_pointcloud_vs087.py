#!/usr/bin/env python
import numpy as np
import struct
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import rospy
import rospkg
from sensor_msgs import point_cloud2
from std_msgs.msg import Int32MultiArray, Header
from sensor_msgs.msg import PointCloud2, PointField
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from darknet_rospy_msgs.msg import DarkBoxArray

from darknet_ros_msgs.msg import BoundingBoxes


class DarkPixelTo3DPointNode:

    def __init__(self):
        rospy.init_node('DarkPixelTo3DPointNode', anonymous=True)
        self.target_label = "VS087"
        self.th_z = 0.018
        self.pointcloud_sub = rospy.Subscriber("/kinect_left/qhd/points", PointCloud2, self.pointcloud_callback, queue_size=1, buff_size=2**24)
        self.dark2d_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.dark_callback, queue_size=1, buff_size=2**24)
        self.pointcloud_pub = rospy.Publisher("/darknet/"+self.target_label+"/pointcloud", PointCloud2, queue_size=1)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listner = tf2_ros.TransformListener(self.tf_buffer)
        self.pointcloud = None
        self.run_once_flg = False
        self.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                       PointField('y', 4, PointField.FLOAT32, 1),
                       PointField('z', 8, PointField.FLOAT32, 1),
                       PointField('rgba', 12, PointField.UINT32, 1),]
        self.header = Header()
        self.header.frame_id = "world"
        r = 234
        g = 29
        b = 117
        a = 255
        self.rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
        rospy.loginfo("Initialized DarkPixelTo3DPointNode.")

    def pixelTo3DPoint(self, u, v, pointcloud):
        point_step = pointcloud.point_step
        row_step = pointcloud.row_step
        array_pos = v*row_step + u*point_step

        bytesX = [ord(x) for x in pointcloud.data[array_pos:array_pos+4]]
        bytesY = [ord(x) for x in pointcloud.data[array_pos+4: array_pos+8]]
        bytesZ = [ord(x) for x in pointcloud.data[array_pos+8:array_pos+12]]

        byte_format=struct.pack('4B', *bytesX)
        X_sensor = struct.unpack('f', byte_format)[0]

        byte_format=struct.pack('4B', *bytesY)
        Y_sensor = struct.unpack('f', byte_format)[0]

        byte_format=struct.pack('4B', *bytesZ)
        Z_sensor = struct.unpack('f', byte_format)[0]

        nan_flg = False
        if math.isnan(X_sensor) or \
           math.isnan(Y_sensor) or \
           math.isnan(Z_sensor):
            nan_flg = True

        return [X_sensor, Y_sensor, Z_sensor], nan_flg

    def transform_3D_point_to_world_frame(self, xyz_point_from_sensor, sensor_frame_id):
        pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.header.frame_id = sensor_frame_id
        pose_stamped.pose.orientation.w = 1.0
        pose_stamped.pose.position.x = xyz_point_from_sensor[0]
        pose_stamped.pose.position.y = xyz_point_from_sensor[1]
        pose_stamped.pose.position.z = xyz_point_from_sensor[2]

        transform = self.tf_buffer.lookup_transform("world",
                                                    pose_stamped.header.frame_id,
                                                    rospy.Time(0),
                                                    rospy.Duration(1.0))        
        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)

        X_world = pose_transformed.pose.position.x
        Y_world = pose_transformed.pose.position.y
        Z_world = pose_transformed.pose.position.z

        return [X_world, Y_world, Z_world]

    def extract_3d_points(self, dark_box):
        X = []
        Y = []
        Z = []

        # for u in range(320, 500+1):
        #     for v in range(80, 500+1):

        for u in range(dark_box.xmin, dark_box.xmax+1):
            for v in range(dark_box.ymin, dark_box.ymax+1):
                try:
                    xyz_point_from_sensor, nan_flg = self.pixelTo3DPoint(u,v,self.pointcloud)
                    
                    if nan_flg is not True:
                        xyz_point_from_world = self.transform_3D_point_to_world_frame(xyz_point_from_sensor,
                                                                                      self.pointcloud.header.frame_id)
                        X.append(xyz_point_from_world[0])
                        Y.append(xyz_point_from_world[1])
                        Z.append(xyz_point_from_world[2])
                except:
                    print("")
        return X, Y, Z

    def filter_3d_points(self, X, Y, Z):
        X_ = []
        Y_ = []
        Z_ = []
        pointN = len(X)
        th_min_Z = min(Z) + self.th_z
        for i in range(pointN):
            if Z[i] > th_min_Z:
                X_.append(X[i])
                Y_.append(Y[i])
                Z_.append(Z[i])
        return X_, Y_, Z_

    def pointcloud_callback(self, data):
        self.pointcloud = data

    def dark_callback(self, data):
        if self.pointcloud is None:
            return

        points = []
        for dark_box in data.bounding_boxes:
            if dark_box.Class == self.target_label:
                X, Y, Z = self.extract_3d_points(dark_box)
                X, Y, Z = self.filter_3d_points(X, Y, Z)
                pointN = len(X)
                for i in range(pointN):
                    pt = [X[i], Y[i], Z[i], self.rgb]
                    points.append(pt)

        pc2 = point_cloud2.create_cloud(self.header, self.fields, points)
        self.pointcloud_pub.publish(pc2)            


if __name__ == '__main__':
    DarkPixelTo3DPointNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting Down.")
