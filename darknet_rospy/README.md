roscore
rosparam set /use_simtime true
roslaunch darknet_rospy darknet_rospy_yolo3_kinect_head.launch
rosbag play -l yolo_kinect_2_carrots.bag
rosrun darknet_rospy image_pixel_to_3d_pointcloud2.py
rviz

