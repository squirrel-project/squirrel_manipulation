#include "../include/SimplePathFollowing.hpp"


using namespace std;


SimplePathFollowing::SimplePathFollowing(){

}


void SimplePathFollowing::updatePushPlanner(geometry_msgs::Pose2D pose_robot_, geometry_msgs::PoseStamped pose_object_){
    this->pose_robot_ = pose_robot_;
    this->pose_object_ = pose_object_;
    pose_object_.pose.position.x = pose_robot_.x;
    pose_object_.pose.position.y = pose_robot_.y;

    this-> err_th_toll_ = 0.01;
    this-> err_t_toll_ = 0.01;
    this-> vel_ang_  = 0.3;
    this-> vel_lin_ = 0.3;



}
geometry_msgs::Twist SimplePathFollowing::getVelocities(){
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

    //cout << "robot pose" <<endl << pose_robot_<<endl<<endl;
    //cout << " target pose " <<endl << target_<<endl<<endl;

    //translation error
    double err_lin = distancePoints(pose_robot_.x, pose_robot_.y, target_.pose.position.x, target_.pose.position.y);

    if (err_th < - err_th_toll_) {
        cmd.angular.z =  vel_ang_;
    }
    else if (err_th > err_th_toll_){
        cmd.angular.z =  - vel_ang_;

    }
    else {
        cmd.angular.z = 0.0;
    }

    if (err_lin > err_t_toll_){
        cmd.linear.x = vel_lin_;
    }
    else{
        cmd.linear.x = 0.0;
    }

    //cout << "(simple path follow) err theta"<< err_th<<" lin err "<<err_lin<<endl;
    //cout<< "vel th "<<vel_ang_<< "vel lin"<<vel_lin_<<endl;



    return cmd;

}
