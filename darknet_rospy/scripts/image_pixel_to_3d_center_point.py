#!/usr/bin/env python
import numpy as np
import struct
import math
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
        self.marker_pub = rospy.Publisher("transferred_image_pixel", MarkerArray, queue_size = 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listner = tf2_ros.TransformListener(self.tf_buffer)
        self.pointcloud = None
        rospy.loginfo("Initialized DarkPixelTo3DPointNode.")

    def pixelTo3DPoint(self, u, v, pointcloud):
        point_step = pointcloud.point_step
        row_step = pointcloud.row_step
        array_pos = v*row_step + u*point_step

        bytesX = [ord(x) for x in pointcloud.data[array_pos:array_pos+4]]
        bytesY = [ord(x) for x in pointcloud.data[array_pos+4: array_pos+8]]
        bytesZ = [ord(x) for x in pointcloud.data[array_pos+8:array_pos+12]]

        byte_format=struct.pack('4B', *bytesX)
        X = struct.unpack('f', byte_format)[0]

        byte_format=struct.pack('4B', *bytesY)
        Y = struct.unpack('f', byte_format)[0]

        byte_format=struct.pack('4B', *bytesZ)
        Z = struct.unpack('f', byte_format)[0]

        nan_flg = False
        if math.isnan(X) or \
           math.isnan(Y) or \
           math.isnan(Z):
            nan_flg = True

        return [X, Y, Z], nan_flg

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

        return pose_transformed


    def generate_marker(self, point, index):
        marker_data = Marker()
        marker_data.header.frame_id = point.header.frame_id
        marker_data.header.stamp = rospy.Time.now()
        marker_data.ns = "basic_shapes"
        marker_data.id = index
        marker_data.action = Marker.ADD
        marker_data.pose.position = point.pose.position
        marker_data.pose.orientation = point.pose.orientation
        marker_data.color.r = 1.0
        marker_data.color.g = 0.7
        marker_data.color.b = 0.0
        marker_data.color.a = 1.0
        marker_data.scale.x = 0.01
        marker_data.scale.y = 0.01
        marker_data.scale.z = 0.01
        marker_data.type = 2
        return marker_data

    def pointcloud_callback(self, data):
        self.pointcloud = data

    def dark_callback(self, data):
        if self.pointcloud is None:
            return

        marker_array = MarkerArray()
        marker_list = []
        index = 0

        for dark_box in data.dark_array:
            
            u = int(dark_box.xmin+((dark_box.xmax-dark_box.xmin)/2.0))
            v = int(dark_box.ymin+((dark_box.ymax-dark_box.ymin)/2.0)) 
            xyz_point_from_sensor, nan_flg = self.pixelTo3DPoint(u,v,self.pointcloud)

            if nan_flg is True:
                return

            xyz_point_from_world = self.transform_3D_point_to_world_frame(xyz_point_from_sensor,
                                                                          self.pointcloud.header.frame_id)
            marker_data = self.generate_marker(xyz_point_from_world, index)
            marker_list.append(marker_data)
            index += 1

        marker_array.markers = marker_list
        self.marker_pub.publish(marker_array)


if __name__ == '__main__':
    DarkPixelTo3DPointNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting Down.")
