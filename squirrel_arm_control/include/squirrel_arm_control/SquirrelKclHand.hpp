#ifndef SQUIRREL_KCL_HAND
#define SQUIRREL_KCL_HAND

#include <string>
#include <vector>
#include <armadillo>
#include <ros/ros.h>
#include <kukadu/kukadu.h>
#include <std_msgs/Duration.h>
#include <geometry_msgs/Pose.h>

class SquirrelKclHand : public kukadu::GenericHand {

private:


public:

    SquirrelKclHand();

    virtual void connectHand();

    virtual void closeHand(double percentage, double velocity);

    virtual void disconnectHand();

    virtual std::vector<arma::mat> getTactileSensing();

    virtual std::string getHandName();

};

#endif
