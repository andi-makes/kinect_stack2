This is a freenect based ROS2 Jazzy package for interfacing with a Microsoft Kinect.

The provided `kinect.launch.py` file also publishes a colored pointcloud.

The code was inspired in parts from
- [SriharshaShesham/KinectV1-Ros2](https://github.com/SriharshaShesham/KinectV1-Ros2/tree/ros2-humble-unified) and
- [fadlio/kinect_ros2](https://github.com/fadlio/kinect_ros2).
Thanks for your efforts!

# Installation

Install the [libfreenect](https://github.com/OpenKinect/libfreenect) library from source.
If you want tilt control to work, build with `cmake .. -DBUILD_REDIST_PACKAGE=OFF`.
You may need to manually upload the new firmware to the kinect.

Add this package to your workspace and build it using colcon.
