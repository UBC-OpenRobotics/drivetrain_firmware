#ifndef DRIVETRAIN_ROS_NODE_H
#define DRIVETRAIN_ROS_NODE_H

#include <math.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

//comms constants
#define CMD_VEL_TOPIC "/cmd_vel_mux"
#define LOG_PUB_TOPIC "/esp32/log"

//robot kinematics constants
const float WHEEL_RADIUS = 0.075;
const float DRIVETRAIN_WIDTH = 0.55;

//unit conversions
const float RADS_TO_RPM = 9.54929658551;


/** \brief Software interface for control over rosserial*/
class DriveTrainControlInterface
{
public:
    ros::Publisher logger;
    ros::NodeHandle nh;
    ros::Subscriber<geometry_msgs::Twist, DriveTrainControlInterface> twistSubscriber;

    DriveTrainControlInterface()
    : twistSubscriber(CMD_VEL_TOPIC, &DriveTrainControlInterface::twistCmdCallback, this),
        logger(LOG_PUB_TOPIC, &string_msg)
    
    {  // Constructor
        cmdUpdateFlag = false;
        velocityCmd[0] = 0;
        velocityCmd[1] = 0;
        _rpmCmd[0] = 0;
        _rpmCmd[1] = 0;
        _dirCmd[0] = true;
        _dirCmd[1] = true;

        nh.initNode();
        nh.subscribe(twistSubscriber);
        nh.advertise(logger);
        // ros::Publisher p("/esp32/log", &log);
    }

    void log(const char * msg){
        string_msg.data = msg;
        logger.publish(&string_msg);
    }

    /**
     * \brief Calculates the desired angular velocity for each wheel based on the kinematic model for the drivetrain
     * see https://en.wikipedia.org/wiki/Differential_wheeled_robot
     * modifies input array
     * \param float[] velocityCmd array, contains desired vecloity and angular velocity for drivetrain
     * \param u_int16_t[] rpmCmd array, function will fill this array with target RPM values for each wheel
     * \param bool[] dirCmd array, function will fill this array with target rotational direction for each wheel
     */
    static void calculateRobotKinematics(float velocityCmd[], u_int16_t rpmCmd[], bool dirCmd[])
    {
        float wR = (velocityCmd[0] + velocityCmd[1] * DRIVETRAIN_WIDTH/2)/WHEEL_RADIUS;
        float wL = (velocityCmd[0] - velocityCmd[1] * DRIVETRAIN_WIDTH/2)/WHEEL_RADIUS;

        dirCmd[0] = wR > 0;
        dirCmd[1] = wL > 0;

        rpmCmd[0] = (u_int16_t) floor(wR*RADS_TO_RPM);
        rpmCmd[1] = (u_int16_t) floor(wL*RADS_TO_RPM);
    }

    /**
     * \brief Checks if the velocity command has been updated since last getVelocityCmd call
     * \return bool
     */
    bool isUpdated()
    {
        return cmdUpdateFlag;
    }

    /**
     * \brief Gets the latest RPM and dir for each motor
     * \param u_int16_t[] rpmCmd - array to be modified with new rpm values
     * \param bool[] dirCmd - array to be modified with rot. direction values
     * 
     * rpmCmd[0] = RPM for right wheel
     * rpmCmd[1] = RPM for left wheel
     * 
     * dirCmd[0] = rot. direction for right wheel (true = forward)
     * dirCmd[1] = rot. direction for left wheel (true = forward)
     */
    void getMotorCmd(u_int16_t rpmCmd[], bool dirCmd[])
    {
        memcpy(rpmCmd, _rpmCmd, sizeof(_rpmCmd));
        memcpy(dirCmd, _dirCmd, sizeof(_dirCmd));
        cmdUpdateFlag = false;
    }

private:
    bool cmdUpdateFlag;
    float velocityCmd[2];
    u_int16_t _rpmCmd[2];
    bool _dirCmd[2];
    std_msgs::String string_msg;

    /**
     * \brief ROS subscriber callback function, to be called whenever a new message is received
     * \param geometry_msgs::Twist message input for callback
     */
    void twistCmdCallback(const geometry_msgs::Twist &msg)
    {
        velocityCmd[0] = msg.linear.x; // in m/s
        velocityCmd[1] = msg.angular.z; // in rad/s
        calculateRobotKinematics(velocityCmd, _rpmCmd, _dirCmd);
        cmdUpdateFlag = true;
        log("twist cmd received");
    }
};

#endif