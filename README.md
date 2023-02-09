# drivetrain_firmware
Firmware for the OB1 bot drivetrain (for the ESP32 wrover-kit).

Uses [PlatformIO](https://platformio.org/) and arduino framework. Depends on [rosserial](http://wiki.ros.org/rosserial) package for arduino.

**What's the firmware for?**
* Receive commands and send sensor data to and from ros master
* Drive the two motors of the drivetrain
* Gather encode measurements
* tbd

Uses rosserial to establish ESP32 as a ros node over UART. The ros master will send a ROS [geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html) message.

The platformIO project is setup for the ESP32 WROVER. We should be able to compile and upload code directly over microUSB. 

Please follow the setup instructions and **always push code to your own branches, NOT the main branch.**

# Drivetrain Kinematics and Control

https://en.wikipedia.org/wiki/Differential_wheeled_robot

Measurements taken from CAD assembly of drivetrain for now.

# Setup Instructions

## 1. Install System Dependencies

You will need:
* [git](https://git-scm.com/)
* [VSCode](https://code.visualstudio.com/)
* [PlatformIO extension for VSCODE](https://platformio.org/platformio-ide)

## 2. Clone

Run:

```
git clone <repo link>
```

## 3. Create a branch

```
git branch -b 'new branch name'
```

## 4. Directory Structure (where to put your files)

```
drivetrain_firmware/
├── include
├── lib
├── src
└── test
```

`include` : where we will store our header (.h or .hpp files here!)

`src`: where we will store our .c and .cpp here! 

# Compiling and Uploading Code

pending...

