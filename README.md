# darknet_ros

## Requirements
- Ubuntu 16.04
- ROS kinetic
- GPU (NVIDIA GTX1070 is confirmed)
- CUDA 8.0 (or higher)
- CuDNN 6 (or higer)
- OpenCV 3.3.1 (or higer)

## Initial build of this repository
```
$ cd <your_workspace>/src
$ git clone --recursive git@github.com:TakakiNishio/darknet_ros.git
$ cd darknet_ros/darknet && make -j4
$ cd <your_workspace>
$ catkin build
```

## Running an Example
```
$ rosrun darknet_ros darknet_webcam.py
```
