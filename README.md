# Astra

[install]
```
$ sudo apt-get install ros-kinetic-rgbd-launch ros-kinetic-libuvc-camera
```

[repo]
```
$ git clone https://github.com/orbbec/ros_astra_camera
$ git clone https://github.com/ROBOTIS-GIT/ros_astra_launch.git
```

[init]
- roscd astra_camera && ./scripts/create_udev_rules

[launch]
```
$ sudo chmod a+rw /dev/bus/usb/${USB}/${PORT}
$ roslaunch open_manipulator_camera astra_pro.launch
```

# realsense d435

[install]

https://github.com/intel-ros/realsense

[firmware_update]
https://downloadcenter.intel.com/download/28481/Latest-Firmware-for-Intel-RealSense-D400-Product-Family?v=t
I have installed 5.11.1.0

[repo]

```
$ sudo apt-get install ros-kinetic-rgbd-launch
$ sudo apt-get install librealsense2-dev
$ git clone https://github.com/intel-ros/realsense.git
```

[launch]
```
$ roslaunch realsense2_camera rs_camera.launch
```

# Raspberry Pi CameraV2

[repo]
```
$ git clone https://github.com/ROBOTIS-GIT/open_manipulator_perceptions.git
```

[launch - Raspberry Pi]
```
$ roslaunch open_manipulator_camera rpicamera.launch
```

# ZED (Not supported in ubuntu 16.04)
[install]

http://wiki.ros.org/zed-ros-wrapper
https://www.stereolabs.com/developers/release/
https://developer.nvidia.com/cuda-downloads  (select x86_64 -> deb(local) and follow instructions that you can find download button below)

```
$ sudo apt-get install libpcl1
$ sudo apt-get install ros-kinetic-pcl-ros
$ sudo apt install libpng16-16
```

[repo]
```
$ git clone https://github.com/stereolabs/zed-ros-wrapper
```

[launch]
```
$ roslaunch zed_wrapper zed.launch
```

[error]
glbc2.7 is not supported in xenial

# AR marker

[repo]
```
$ git clone https://github.com/ROBOTIS-GIT/open_manipulator_perceptions.git
```

[launch - astra pro]
```
$ roslaunch open_manipulator_ar_markers ar_pose.launch use_gazebo:=false camera_model:=astra_pro
```

[launch - realsense d435]
```
$ roslaunch open_manipulator_ar_markers ar_pose.launch use_gazebo:=false camera_model:=realsense_d435
```

[launch - Raspberry Pi CameraV2]
```
[Raspberry Pi]
$ roslaunch open_manipulator_camera rpicamera.launch

[Remote PC]
$ roslaunch open_manipulator_ar_markers ar_pose.launch use_gazebo:=false camera_model:=raspicam
```





