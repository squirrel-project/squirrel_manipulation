#include "../include/BangBangPush.hpp"

using namespace std;

BangBangPush::BangBangPush():
    PushPlanner()
{
    private_nh.param("push/error_theta_tolerance", err_th_toll_,0.1);
    private_nh.param("push/error_target_tolerance", err_t_toll_ ,0.01);
    private_nh.param("push/velocity_angular_max", vel_ang_max_ ,0.2);
    private_nh.param("push/velocity_linear_max", vel_lin_max_ ,0.3);
    private_nh.param("push/velocity_y_max", vel_y_max_ ,0.3);
}

geometry_msgs::Twist BangBangPush::getVelocities(){
    //initialize value
    geometry_msgs::Twist cmd = getNullTwist();

    //translation error object-target
    if (dO2P > err_t_toll_){
        cmd.linear.x = vel_lin_max_;
    }
    else{
        cmd.linear.x = 0.0;
    }

    //orientation error
    double err_th = rotationDifference(aO2P, pose_robot_.theta);

    if (err_th < - err_th_toll_) {
        // rotating left
        cmd.angular.z =  vel_ang_max_;
    }
    else if (err_th > err_th_toll_){
        // rotating right
        cmd.angular.z =  - vel_ang_max_;
    }
    else {
        cmd.angular.z = 0.0;
    }

    //the angle object-robot-target
    if (aORP < - err_th_toll_) {
        // moving left
        cmd.linear.y = vel_y_max_;
    }
    else if (aORP > err_th_toll_){
        // moving right
        cmd.linear.y = - vel_y_max_;
    }
    else {
        cmd.linear.y = 0.0;
    }


    return cmd;

}

void BangBangPush::initChild() {}
