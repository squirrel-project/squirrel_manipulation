#ifndef SIMPLEPATHFOLLOWING_HPP
#define SIMPLEPATHFOLLOWING_HPP

#include "PushPlanner.hpp"

#include <geometry_msgs/Pose2D.h>

class SimplePathFollowing : public PushPlanner{

protected:
    void initChild();

public:
    SimplePathFollowing();
    void updatePushPlanner(geometry_msgs::Pose2D pose_robot_, geometry_msgs::PoseStamped pose_object_) override;
    geometry_msgs::Twist getVelocities();

};

#endif // SIMPLEPATHFOLLOWING_HPP
