# ROS Serial Arduino library

## Purpose

Installing ROS Serial library into and Arduino IDE can be an involved task. See instructions http://wiki.ros.org/rosserial. If you don't already have ROS installed on your workstation it's difficult to build due to the Catkin workspace requirement. This repo packages the library for the Arduino IDE and PlatformIO in an expected format.

## ROS Serial Description

rosserial is a protocol for wrapping standard ROS serialized messages and multiplexing multiple topics and services over a character device such as a serial port or network socket.

Use an Arduino as a ROS publisher/subscriber

Works with http://wiki.ros.org/rosserial, requires a rosserial node to connect

## Updating

There is a script to pull in the current changes from https://github.com/ros-drivers/rosserial. It creates a Docker container with ROS and Catkin setup. It then downloads the current version of ROS Serial from Github and builds the libraries. Finally it overwrites the current src files with the new build. Currently there is no automatic testing. So it must be manually tested (to involved to discuss here).

```
./update.sh
```

## Reporting issues

If this repo is out of date feel free to report an issue. However pull requests should be directed to https://github.com/ros-drivers/rosserial. Any changes to this repo would be overwritten by incoming changes from upstream.

## License

Apache 2.0

## Author Information

Joshua Frank [@frankjoshua77](https://www.twitter.com/@frankjoshua77)
<br>
[http://roboticsascode.com](http://roboticsascode.com)
