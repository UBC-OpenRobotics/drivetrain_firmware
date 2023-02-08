#include <Arduino.h>
#include <drivetrain_ros_node.h>

DriveTrainControlInterface d;

void setup() {
  // put your setup code here, to run once:
  d = DriveTrainControlInterface();
}

void loop() {
  d.nh.spinOnce();
  if(d.isUpdated()){
    float vel[2];
    d.getVelocityCmd(vel);
  }
  // put your main code here, to run repeatedly:
}