#ifndef DRIVETRAIN_ROS_NODE_H
#define DRIVETRAIN_ROS_NODE_H

#include <ros.h>
#include <geometry_msgs/Twist.h>

#define CMD_VEL_TOPIC "/cmd_vel"


/** \brief Software interface for control over rosserial*/
class DriveTrainControlInterface
{
public:
    
    ros::NodeHandle nh;
    ros::Subscriber<geometry_msgs::Twist, DriveTrainControlInterface> twistSubscriber;

    DriveTrainControlInterface()
    : twistSubscriber(CMD_VEL_TOPIC, &DriveTrainControlInterface::twistCmdCallback, this)
    {  // Constructor
        cmdUpdateFlag = false;
        velocityCmd[0] = 0.0;
        velocityCmd[1] = 0.0;
        nh.initNode();
        nh.subscribe(twistSubscriber);
    };

    /**
     * \brief Checks if the velocity command has been updated since last getVelocityCmd call
     * \return bool
     */
    bool isUpdated()
    {
        return cmdUpdateFlag;
    }

    /**
     * \brief Gets the latest velocity command in linear x and angular z
     * \param float[] input array to be modified
     * 
     * vel[0] = linear x velocity
     * vel[1] = linear y velocity 
     */
    void getVelocityCmd(float vel[])
    {
        vel[0] = velocityCmd[0];
        vel[1] = velocityCmd[1];
        cmdUpdateFlag = false;
    }


private:
    bool cmdUpdateFlag;
    float velocityCmd[2];

    /**
     * \brief ROS subscriber callback function, to be called whenever a new message is received
     * \param geometry_msgs::Twist message input for callback
     */
    void twistCmdCallback(const geometry_msgs::Twist &msg)
    {
        cmdUpdateFlag = true;
        velocityCmd[0] = msg.linear.x;
        velocityCmd[1] = msg.angular.z;
    }
};  

#endif