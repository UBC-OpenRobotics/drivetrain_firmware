#include <drivetrain_ros_node.h>

void DrivetrainRosNode::twistCmdCb( const geometry_msgs::Twist &msg){
    this->cmdUpdateFlag = true;
    
    this->velocityCmd[0] = msg.linear.x;
    this->velocityCmd[1] = msg.angular.z;
}