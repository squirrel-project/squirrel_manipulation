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
    this->pose_object_.pose.position.x = pose_robot_.x;
    this->pose_object_.pose.position.y = pose_robot_.y;

}
geometry_msgs::Twist SimplePathFollowing::getVelocities(){
    //initialize value
    geometry_msgs::Twist cmd = getNullTwist();

    geometry_msgs::PoseStamped target_ = this->getLookaheadPoint();

    //angle between robot pose and target
    double aR2P = atan2(target_.pose.position.y - pose_robot_.y, target_.pose.position.x - pose_robot_.x);
    if (isnan(aR2P)) aR2P = 0;

    //orientation error
    double err_th = pose_robot_.theta - aR2P;

    //translation error
    double err_lin = distancePoints(pose_robot_.x, pose_robot_.y, target_.pose.position.x, target_.pose.position.y);

    if (err_lin < 0.05){
        goal_reached_ = true;
        cmd.angular.z = 0.0;
        cmd.linear.x = 0.0;
        return cmd;

    }

    if (err_th < - err_th_toll_) {
        cmd.angular.z =  vel_ang_max_;
    }
    else if (err_th > err_th_toll_){
        cmd.angular.z =  - vel_ang_max_;

    }
    else {
        cmd.angular.z = 0.0;
    }

    if (err_lin > err_t_toll_){
        cmd.linear.x = vel_lin_max_;
    }
    else{
        cmd.linear.x = 0.0;
    }

    return cmd;

}
