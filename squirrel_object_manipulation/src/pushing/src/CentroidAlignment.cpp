#include "../include/CentroidAlignment.hpp"

using namespace std;
using namespace arma;


CentroidAlignment::CentroidAlignment():
    PushPlanner()
{
    private_nh.param("push/error_theta_tolerance", err_th_toll_,0.01);
    private_nh.param("push/error_target_tolerance", err_t_toll_ ,0.01);
    private_nh.param("push/velocity_angular_max", vel_ang_max_ , 0.6);
    private_nh.param("push/velocity_linear_max", vel_lin_max_ , 0.8);
    private_nh.param("push/velocity_y_max", vel_y_max_ , 0.8);

}


geometry_msgs::Twist CentroidAlignment::getVelocities(){

    double sigma_gc = 0.1;
    double sigma_c = 0.2;

    //initialize value
    geometry_msgs::Twist cmd = getNullTwist();

    // error object-target
    vec object_error_(2);
    object_error_(0) = current_target_.pose.position.x - pose_object_.pose.position.x;
    object_error_(1) = current_target_.pose.position.y - pose_object_.pose.position.y;

    //robot displacement from line object-target
    vec displacement_point_ = closestPointOnLine(pose_robot_.x, pose_robot_.y, pose_object_.pose.position.x, pose_object_.pose.position.y, current_target_.pose.position.x, current_target_.pose.position.y);

    vec robot_displacement_(2);
    robot_displacement_(0) = displacement_point_(0) - pose_robot_.x;
    robot_displacement_(1) = displacement_point_(1) - pose_robot_.y;

    // robot_error_vector = vector sum
    vec u_(2);
    u_(0) = sigma_gc * robot_displacement_(0) +  sigma_c * object_error_(0);
    u_(1) = sigma_gc * robot_displacement_(1) +  sigma_c * object_error_(1);

    // transform to robot frame
    vec u_R_ = rotate2DVector(u_, -pose_robot_.theta);

    cmd.linear.x = u_R_(0);
    cmd.linear.y = u_R_(1);

    return cmd;

}

void CentroidAlignment::initChild(){}
