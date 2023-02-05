#ifndef DRIVETRAIN_ROS_NODE_H
#define DRIVETRAIN_ROS_NODE_H

#include <ros.h>
#include <geometry_msgs/Twist.h>

/** \brief Software interface over rosserial*/
class DrivetrainRosNode
{
public:

    ros::NodeHandle nh;

    /**
     * \brief Constructor for sw interface class
     */
    DrivetrainRosNode();

    /** \brief Initialize the robot hardware interface */
    void init();

    void isUpdated();

    void getVelocityCmd(float* velocityArray[]);


private:
    void twistCmdCb( const geometry_msgs::Twist &msg);


    geometry_msgs::Twist* twist;
    bool cmdUpdateFlag;
    float velocityCmd[2];
    ros::Subscriber<geometry_msgs::Twist> sub;
    // ros::Publisher<define type>* pub;
};  

#endif