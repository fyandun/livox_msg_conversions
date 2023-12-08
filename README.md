# Package for conveting Livox messages

This repo contains code for converting Livox pointcloud2 messages to Livox customized data package format. It is only one way, meaning that the code to conver from customized msg to pointcloud2 is not implemented.

I used the documentation of the [livox ROS driver](https://github.com/Livox-SDK/livox_ros_driver) and the [driver code](https://github.com/Livox-SDK/livox_ros_driver/blob/master/livox_ros_driver/livox_ros_driver/lddc.cpp) to save each message type for getting the correct values for each field.

## Conversion for using in FASTLIO and FASTER-LIO
```
roslaunch livox_msg_conversions convert_to_custom_fastlio.launch
```
## Conversion for using in LIOSAM
Note: This conversion has not been thoroughly tested yet.
```
roslaunch livox_msg_conversions convert_to_custom_liosam.launch
```
