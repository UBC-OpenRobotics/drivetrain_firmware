#include <drivetrain_motors.h>
#include <drivetrain_ros_node.h>

DriveTrainControlInterface d;
u_int16_t rpmCmd[2];
bool dirCmd[2];

void setup()
{
  // put your setup code here, to run once:
  d = DriveTrainControlInterface(d);
  setupMotors();
  disableMotors();
  d.nh.logdebug("[ESP32] Setup complete.");
}

void loop()
{
  if (d.isUpdated())
  {
    d.getMotorCmd(rpmCmd, dirCmd);
    if (rpmCmd[0] == 0 and rpmCmd[1] == 0){
      disableMotors();
      d.log("disabled motors");
    } else{
      enableMotors();
      d.log("enabled motors");
      driveMotors(rpmCmd, dirCmd);
    }
    d.log("motor cmd sent");
  }

  d.nh.spinOnce();
}
