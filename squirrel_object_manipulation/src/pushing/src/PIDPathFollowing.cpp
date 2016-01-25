#include "../include/PIDPathFollowing.hpp"

using namespace std;

PIDPathFollowing::PIDPathFollowing():
    PushPlanner()
{
    private_nh.param("push/proportional_x", p_x_, 0.5);
    private_nh.param("push/derivative_x", d_x_, 0.1);
    private_nh.param("push/integral_x", i_x_, 0.2);
    private_nh.param("push/integral_x_max", i_x_max_, 1.0);
    private_nh.param("push/integral_x_min", i_x_min_, -1.0);
    private_nh.param("push/proportional_theta", p_theta_, 1.0);
    private_nh.param("push/derivative_theta", d_theta_, 0.2);
    private_nh.param("push/integral_theta", i_theta_, 0.1);
    private_nh.param("push/integral_theta_max", i_theta_max_, 1.0);
    private_nh.param("push/integral_theta_min", i_theta_min_, -1.0);
}

void PIDPathFollowing::updatePushPlanner(geometry_msgs::Pose2D pose_robot_, geometry_msgs::PoseStamped pose_object_){
    this->pose_robot_ = pose_robot_;
    this->pose_object_ = pose_object_;
    this->pose_object_.pose.position.x = pose_robot_.x;
    this->pose_object_.pose.position.y = pose_robot_.y;

}

void PIDPathFollowing::initChild() {

    pid_x_.initPid(p_x_, i_x_, d_x_, i_x_max_, i_x_min_);
    pid_theta_.initPid(p_theta_, i_theta_, d_theta_, i_theta_max_, i_theta_min_);
}

geometry_msgs::Twist PIDPathFollowing::getVelocities(){
    //initialize value
    geometry_msgs::Twist cmd = getNullTwist();

    //angle between robot pose and target
    double aR2P = atan2(current_target_.pose.position.y - pose_robot_.y, current_target_.pose.position.x - pose_robot_.x);
    if (isnan(aR2P)) aR2P = 0;

    //orientation error
    double err_th = aR2P - pose_robot_.theta;

    //translation error
    double err_lin = distancePoints(pose_robot_.x, pose_robot_.y, current_target_.pose.position.x, current_target_.pose.position.y);

    if (err_lin < 0.05){
        goal_reached_ = true;
        cmd.angular.z = 0.0;
        cmd.linear.x = 0.0;
        return cmd;

    }

    cmd.angular.z =  pid_theta_.computeCommand(err_th, ros::Duration(time_step_));
    cmd.linear.x = pid_x_.computeCommand(err_lin, ros::Duration(time_step_));

    return cmd;

}
