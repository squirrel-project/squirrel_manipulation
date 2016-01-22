#ifndef PUSHRECOVERYSM_H
#define PUSHRECOVERYSM_H

#include "PushPlanner.hpp"

#include <geometry_msgs/Pose2D.h>

class PushRecoverySM : public PushPlanner {
public:
    PushRecoverySM();
    void updatePushPlanner(geometry_msgs::Pose2D pose_robot_, geometry_msgs::PoseStamped pose_object_);
    geometry_msgs::Twist getVelocities();

};

#endif // SIMPLEPUSH_H
