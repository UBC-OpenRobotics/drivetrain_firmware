#include <drivetrain_motors.h>
#include <drivetrain_ros_node.h>

DriveTrainControlInterface d;
u_int16_t rpmCmd[2];
bool dirCmd[2];

void setup() {
  // put your setup code here, to run once:
  d = DriveTrainControlInterface();
  setup_motors();
  enableMotor();
}

void loop() {
  d.nh.spinOnce();
  
  if(d.isUpdated()){
    d.getMotorCmd(rpmCmd, dirCmd);
    driveMotor(rpmCmd[0], dirCmd[0]);
  }
}
 
