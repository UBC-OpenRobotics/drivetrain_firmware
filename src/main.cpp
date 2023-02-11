#include <drivetrain_motors.h>
#include <drivetrain_ros_node.h>

DriveTrainControlInterface d;
u_int16_t rpmCmd[2];
bool dirCmd[2];

void setup()
{
  // put your setup code here, to run once:
  d = DriveTrainControlInterface(d);
  setup_motors();
  enableMotor();
  d.log("esp32 setup complete");
}

void loop()
{
  if (d.isUpdated())
  {
    d.getMotorCmd(rpmCmd, dirCmd);
    driveMotor(rpmCmd, dirCmd);
    d.log("motor cmd sent");
  }

  d.nh.spinOnce();
}
