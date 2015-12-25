#include "../include/PushRecoverySM.hpp"

using namespace std;

PushRecoverySM::PushRecoverySM()
{
}


void PushRecoverySM::updatePushPlanner(geometry_msgs::Pose2D pose_robot_, geometry_msgs::PoseStamped pose_object_){
    this->pose_robot_ = pose_robot_;
    this->pose_object_ = pose_object_;

    this-> err_th_toll_ = 0.01;
    this-> err_t_toll_ = 0.01;
    this-> vel_ang_  = 0.3;
    this-> vel_lin_ = 0.3;
    this-> vel_y = 0.1;


}
geometry_msgs::Twist PushRecoverySM::getVelocities(){
    //initialize value
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;


    geometry_msgs::PoseStamped target_ = this->getLookaheadPoint();

    //angle between robot pose and target
    double aR2P = atan2(target_.pose.position.y - pose_robot_.y, target_.pose.position.x - pose_robot_.x);
    if(aR2P>3.14) aR2P = aR2P - 3.14;
    if(aR2P<-3.14) aR2P = aR2P + 3.14;
    if (isnan(aR2P)) aR2P = 0;

    //orientation error
    double err_th = pose_robot_.theta - aR2P;

    //translation error object-target
    double dO2T = distancePoints(pose_object_.pose.position.x, pose_object_.pose.position.y, target_.pose.position.x, target_.pose.position.y);

    //translation error object-robot
    double dO2R = distancePoints(pose_object_.pose.position.x, pose_object_.pose.position.y, pose_robot_.x, pose_robot_.y);

    //translation error object-robot
    double dR2T = distancePoints(target_.pose.position.x, target_.pose.position.y, pose_robot_.x, pose_robot_.y);

    //angle object-robot-target

    double aORT=acos((dR2T*dR2T-dO2T*dO2T+dO2R*dO2R)/(2*dO2R*dR2T));
    if (aORT > 3.14) aORT = aORT - 2*3.14;
    if (aORT < -3.14) aORT = aORT + 2*3.14;
    if (isnan(aORT)) aORT = 0;

    if (dO2T < 0.05){
        goal_reached_ = true;
        cmd.angular.z = 0.0;
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        return cmd;

    }

    if (err_th < - err_th_toll_) {
        cmd.angular.z =  vel_ang_;
        cmd.linear.y = vel_y;
    }
    else if (err_th > err_th_toll_){
        cmd.angular.z =  - vel_ang_;
        cmd.linear.y = -vel_y;

    }
    else {
        cmd.angular.z = 0.0;
        cmd.linear.y = 0.0;
    }

    if (dO2T  > err_t_toll_){
        cmd.linear.x = vel_lin_;
    }
    else{
        cmd.linear.x = 0.0;
    }

    return cmd;


}
