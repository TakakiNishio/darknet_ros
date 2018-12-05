AR Marker Tracking
====================
## Requirements
- [ar_track_alvar](http://wiki.ros.org/ar_track_alvar) 
- [libuvc_camera](http://wiki.ros.org/libuvc_camera) (for USB camera)

## Generating AR marker
```
$ rosrun ar_track_alvar createMarker -s 4.5 0
```
## Examples
- web camera
  ```
  $ sudo chmod o+w /dev/bus/usb/001/006
  $ roslaunch ar_marker_tracking ar_marker_human_side_camera.launch
  ```
- kinect
  ```
  $ roslaunch kinect2_bridge kinect2_bridge.launch base_name:=kinect_head depth_method:=cpu
  ```
  ```
  $ roslaunch teaching_booth teaching_booth.launch
  ```
  ```
  $ roslaunch ar_marker_tracking ar_marker_human_side_kinect.launch
  ```
## Teaching by Demonstration (Prototype)
### 1. Recording Human Motion Phase
```
$ roslaunch ar_marker_tracking ar_marker_human_side_camera.launch
```
```
$ roslaunch ar_marker_tracking record_trajectory.launch target_frame:=part_0 reference_frame:=parts_pallete output_file:=trajectory_0001.txt
```
```
$ rosrun ar_marker_tracking keyboard_switch.py
```
```
$ roscd ar_marker_tracking/scripts/
$ python3 trajectory_classification_python3.py -f trajectory_0001.txt
```
## 2. Transform Trajectory Phase 
(`parts_pallete` -> `world`)
```
$ roslaunch denso_robot_bringup vs087_and_mhand_bringup.launch planner:=stomp
```
```
$ roslaunch ar_marker_tracking ar_marker_robot_side_sim_camera.launch 
```
```
$ roslaunch ar_marker_tracking transform_trajectory.launch input_file:=trajectory_0001.txt
```
## 3. Generating and Executing Robot Motion Phase 
```
$ roslaunch denso_robot_bringup vs087_and_mhand_bringup.launch planner:=stomp
```
```
$ roslaunch ar_marker_tracking ar_marker_robot_side_sim_camera.launch
```
```
$ rosrun ik_solver ik_stomp
```
```
$ rosrun ar_marker_tracking generate_initial_trajectory_for_moving.py 
```
```
$ roslaunch ar_marker_tracking trajectory_classification.launch input_file:=robot_trajectory_0001.txt 
```

## References for AR marker
- http://ishi.main.jp/ros/ros_ar_indiv.html
- http://ishi.main.jp/ros/ros_ar_bundle.html
