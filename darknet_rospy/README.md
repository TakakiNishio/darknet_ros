# darknet_rospy

## vs087_detection

```
$ roscore
```
```
$ rosparam set /use_sim_time true
$ rviz
```
(open config -> /darknet_rospy/rviz/darknet_kinect_vs087.rviz)
```
$ rosbag play -l denso_back_tf.bag
```
```
$ roslaunch darknet_ros denso_back.launch
```
```
$ rosrun darknet_rospy image_pixel_to_3d_pointcloud_vs087.py
```

