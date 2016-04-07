#ifndef CENTROIDALIGNMENT_H
#define CENTROIDALIGNMENT_H

#include "PushPlanner.hpp"

#include <geometry_msgs/Pose2D.h>
#include <control_toolbox/pid.h>


class CentroidAlignment : public PushPlanner {

private:
    ros::NodeHandle nh;

public:

    CentroidAlignment();
    geometry_msgs::Twist getVelocities();
    void initChild();
    void updateChild(){}


};


#endif // CentroidAlignment_H
