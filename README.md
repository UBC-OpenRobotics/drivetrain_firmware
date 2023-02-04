# drivetrain_firmware
Firmware for the OB1 bot drivetrain (for the ESP32 wrover-kit).

Uses [PlatformIO](https://platformio.org/) and arduino framework. Depends on [rosserial](http://wiki.ros.org/rosserial) package for arduino.

Uses rosserial to establish ESP32 as a ros node over UART. The ros master will send a ROS [geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html) message.
