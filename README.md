# Astra

[install]
```
$ sudo apt-get install ros-kinetic-rgbd-launch ros-kinetic-libuvc-camera
```

[repo]
- https://github.com/orbbec/ros_astra_camera
- https://github.com/clearpathrobotics/ros_astra_launch/tree/upstream

[init]
- roscd astra_camera && ./scripts/create_udev_rules

[launch]
```
$ sudo chmod a+rw /dev/bus/usb/${USB}/${PORT}
```

# ZED
[install]

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

# realsense d435

