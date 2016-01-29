#ifndef SQUIRREL_KCL_HAND
#define SQUIRREL_KCL_HAND

#include <string>
#include <vector>
#include <armadillo>
#include <ros/ros.h>
#include <kukadu/kukadu.h>
#include <std_msgs/Duration.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64MultiArray.h>

class SquirrelKclHand : public kukadu::GenericHand {

private:

    static const int numOfFingers = 3;
    static const int tactileSensorsPerFinger = 5;

    const std::string sensorTopic;

    ros::Subscriber sensorSub;

    arma::vec tactile;

    kukadu_mutex tactileMutex;

    KUKADU_SHARED_PTR<ros::NodeHandle> node;

    void sensorCallback(std_msgs::Float64MultiArray sense);

public:

    SquirrelKclHand(KUKADU_SHARED_PTR<ros::NodeHandle> node);

    virtual void connectHand();

    virtual void closeHand(double percentage, double velocity);

    virtual void disconnectHand();

    virtual std::vector<arma::mat> getTactileSensing();

    virtual std::string getHandName();

};

#endif
