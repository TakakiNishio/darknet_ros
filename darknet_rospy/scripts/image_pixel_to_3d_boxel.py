#!/usr/bin/env python
import numpy as np
import struct
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import rospy
import rospkg
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import PointCloud2
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from darknet_rospy_msgs.msg import DarkBoxArray


class DarkPixelTo3DPointNode:

    def __init__(self):
        rospy.init_node('DarkPixelTo3DPointNode', anonymous=True)
        self.pointcloud_sub = rospy.Subscriber("/kinect_head/qhd/points", PointCloud2, self.pointcloud_callback, queue_size=1, buff_size=2**24)
        self.dark2d_sub = rospy.Subscriber("/darknet/recognition_result", DarkBoxArray, self.dark_callback, queue_size=1, buff_size=2**24)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listner = tf2_ros.TransformListener(self.tf_buffer)
        self.pointcloud = None
        self.run_once_flg = False
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
        for u in range(dark_box.xmin, dark_box.xmax+1):
            for v in range(dark_box.ymin, dark_box.ymax+1):
                xyz_point_from_sensor, nan_flg = self.pixelTo3DPoint(u,v,self.pointcloud)

                if nan_flg is not True:
                    xyz_point_from_world = self.transform_3D_point_to_world_frame(xyz_point_from_sensor,
                                                                                  self.pointcloud.header.frame_id)
                    X.append(xyz_point_from_world[0])
                    Y.append(xyz_point_from_world[1])
                    Z.append(xyz_point_from_world[2])

        return X, Y, Z

    def plot_boxel(self, x, y, z, label, index):
        fig = plt.figure(1)
        ax = fig.gca(projection='3d')
        ax.set_xlabel(r'$x$ [m]',fontsize=14)
        ax.set_ylabel(r'$y$ [m]',fontsize=14)
        ax.set_zlabel(r'$z$ [m]',fontsize=14)
        # ax.set_xlim(0, 0.5)
        # ax.set_ylim(-0.25, 0.25)
        ax.set_zlim(-0.1, 0)
        
        ax.plot([0], [0], [0], 'o', color='y', ms=6 , label='world origin')
        ax.plot(x, y, z, "o", color='c', ms=3, mew=0.5, label='extracted pointcloud')
        ax.legend()

        fig.suptitle('detected object: '+label+' ['+str(index)+']' )
        plt.tight_layout()

    def pointcloud_callback(self, data):
        self.pointcloud = data

    def dark_callback(self, data):

        if self.pointcloud is None:
            return

        if self.run_once_flg is True:
            plt.pause(0.01)
            return
        else:
            self.run_once_flg = True
            index = 0
            dark_box = data.dark_array[index]
            X, Y, Z = self.extract_3d_points(dark_box)
            self.plot_boxel(X,Y,Z, dark_box.label, index)
            plt.pause(0.01)


if __name__ == '__main__':
    DarkPixelTo3DPointNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting Down.")
