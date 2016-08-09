#ifndef BANGBANGPUSH_H
#define BANGBANGPUSH_H

#include "PushPlanner.hpp"

#include <geometry_msgs/Pose2D.h>

class BangBangPush : public PushPlanner {
private:
    ros::NodeHandle nh;

public:
    BangBangPush();
    geometry_msgs::Twist getVelocities();
    void initChild();

};


#endif // BANGBANGPUSH_H
