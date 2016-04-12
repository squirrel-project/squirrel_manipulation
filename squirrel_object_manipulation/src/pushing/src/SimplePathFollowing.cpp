#include "../include/SimplePathFollowing.hpp"

using namespace std;

SimplePathFollowing::SimplePathFollowing():
    PushPlanner()
{
    private_nh.param("push/error_theta_tolerance", err_th_toll_,0.01);
    private_nh.param("push/error_target_tolerance", err_t_toll_ ,0.01);
    private_nh.param("push/velocity_angular_max", vel_ang_max_ ,0.3);
    private_nh.param("push/velocity_linear_max", vel_lin_max_ ,0.3);
}

void SimplePathFollowing::initChild() {}

void SimplePathFollowing::updatePushPlanner(geometry_msgs::Pose2D pose_robot_, geometry_msgs::PoseStamped pose_object_){

    this->pose_robot_ = pose_robot_;
    this->pose_object_ = pose_object_;
    geometry_msgs::PoseStamped current = pose_object_;

    current.pose.position.x = pose_robot_.x;
    current.pose.position.y = pose_robot_.y;

    this->current_target_ = this->getLookaheadPoint(current);
    if (visualise_){
        publishMarkerTargetCurrent(current_target_);
        pushing_plan_pub_.publish(pushing_path_);
    }

    //the angle of a vector robot-target
    aR2P = getVectorAngle(current_target_.pose.position.x - pose_robot_.x, current_target_.pose.position.y - pose_robot_.y);

    if (distancePoints(pose_robot_.x, pose_robot_.y, goal_ .pose.position.x, goal_.pose.position.y) < 0.1){
        goal_reached_ = true;
        push_active_ = false;

    }

}

geometry_msgs::Twist SimplePathFollowing::getVelocities(){

    //initialize value
    geometry_msgs::Twist cmd = getNullTwist();

    //orientation error
    double err_th = rotationDifference(aR2P, pose_robot_.theta);

    //translation error
    double err_lin = distancePoints(pose_robot_.x, pose_robot_.y, current_target_.pose.position.x, current_target_.pose.position.y);

    if (err_th < - err_th_toll_)
        cmd.angular.z =  vel_ang_max_;
    else if (err_th > err_th_toll_)
        cmd.angular.z =  - vel_ang_max_;
    else
        cmd.angular.z = 0.0;

    if (err_lin > err_t_toll_)
        cmd.linear.x = vel_lin_max_;
    else
        cmd.linear.x = 0.0;

    return cmd;

}
