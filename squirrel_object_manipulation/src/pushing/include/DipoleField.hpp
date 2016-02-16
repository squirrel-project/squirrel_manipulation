#ifndef DIPOLEFIELD_H
#define DIPOLEFIELD_H

#include "PushPlanner.hpp"

#include <geometry_msgs/Pose2D.h>
#include <control_toolbox/pid.h>

class DipoleField : public PushPlanner {

private:
    ros::NodeHandle nh;

public:

    DipoleField();
    geometry_msgs::Twist getVelocities();
    void initChild();


};


#endif // DipoleField_H
