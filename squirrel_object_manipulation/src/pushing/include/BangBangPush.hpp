#ifndef BANGBANGPUSH_H
#define BANGBANGPUSH_H

#include "PushPlanner.hpp"

#include <geometry_msgs/Pose2D.h>

class BangBangPush : public PushPlanner {
private:
    ros::NodeHandle nh;

public:
    BangBangPush();
    void updatePushPlanner(geometry_msgs::Pose2D pose_robot_, geometry_msgs::PoseStamped pose_object_);
    geometry_msgs::Twist getVelocities();

};


#endif // BANGBANGPUSH_H
