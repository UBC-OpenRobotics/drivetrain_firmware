# drivetrain_firmware
Firmware for the OB1 bot drivetrain (for ESP32).

Uses [PlatformIO](https://platformio.org/) and arduino framework. Depends on [rosserial](http://wiki.ros.org/rosserial) package for arduino. 

The platformIO project is setup for the ESP32 WROVER. We should be able to compile and upload code directly over microUSB. 

Please follow the setup instructions and **always push code to your own branches, NOT the main branch.**

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

