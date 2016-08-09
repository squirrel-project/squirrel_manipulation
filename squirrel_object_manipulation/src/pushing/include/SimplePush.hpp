#ifndef SIMPLEPUSH_H
#define SIMPLEPUSH_H

#include "PushPlanner.hpp"

#include <geometry_msgs/Pose2D.h>

class SimplePush : public PushPlanner {
private:
    ros::NodeHandle nh;

protected:
    void initChild();

public:
    SimplePush();
    geometry_msgs::Twist getVelocities();

};

#endif // SIMPLEPUSH_H
