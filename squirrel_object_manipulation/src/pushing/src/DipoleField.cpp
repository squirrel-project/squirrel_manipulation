#include "../include/DipoleField.hpp"

using namespace std;

DipoleField::DipoleField():
    PushPlanner(){
    private_nh.param("push/error_theta_tolerance", err_th_toll_,0.01);
    private_nh.param("push/error_target_tolerance", err_t_toll_ ,0.01);
    private_nh.param("push/velocity_angular_max", vel_ang_max_ , 0.6);
    private_nh.param("push/velocity_linear_max", vel_lin_max_ , 0.8);
    private_nh.param("push/velocity_y_max", vel_y_max_ , 0.8);
}



geometry_msgs::Twist DipoleField::getVelocities(){

    //initialize value
    geometry_msgs::Twist cmd = getNullTwist();

    //the angle object-robot-target
    double aPOR =  angle3Points(current_target_.pose.position.x, current_target_.pose.position.y, pose_object_.pose.position.x, pose_object_.pose.position.y, pose_robot_.x, pose_robot_.y);

    double gain = 0.5;

    cmd.linear.x = gain * cos(2 * aPOR);
    cmd.linear.y = gain * sin(2 * aPOR);

    return cmd;

}

void DipoleField::initChild() {}
