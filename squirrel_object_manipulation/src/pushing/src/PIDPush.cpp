#include "../include/PIDPush.hpp"

using namespace std;
using namespace arma;

PIDPush::PIDPush():
    PushPlanner()
{
    private_nh.param("push/error_theta_tolerance", err_th_toll_,0.01);
    private_nh.param("push/error_target_tolerance", err_t_toll_ ,0.01);
    private_nh.param("push/velocity_angular_max", vel_ang_max_ , 0.6);
    private_nh.param("push/velocity_linear_max", vel_lin_max_ , 0.2);
    private_nh.param("push/velocity_y_max", vel_y_max_ , 0.2);

    private_nh.param("push/proportional_x", p_x_, 0.5);
    private_nh.param("push/derivative_x", d_x_, 0.1);
    private_nh.param("push/integral_x", i_x_, 0.2);
    private_nh.param("push/integral_x_max", i_x_max_, 1.0);
    private_nh.param("push/integral_x_min", i_x_min_, -1.0);

    private_nh.param("push/proportional_y", p_y_, 0.9);
    private_nh.param("push/derivative_y", d_y_, 0.4);
    private_nh.param("push/integral_y", i_y_, 0.2);
    private_nh.param("push/integral_y_max", i_y_max_, 1.0);
    private_nh.param("push/integral_y_min", i_y_min_, -1.0);

    private_nh.param("push/proportional_theta", p_theta_, 1.0);
    private_nh.param("push/derivative_theta", d_theta_, 0.3);
    private_nh.param("push/integral_theta", i_theta_, 0.3);
    private_nh.param("push/integral_theta_max", i_theta_max_, 1.0);
    private_nh.param("push/integral_theta_min", i_theta_min_, -1.0);
}

void PIDPush::initChild() {

    pid_x_.initPid(p_x_, i_x_, d_x_, i_x_max_, i_x_min_);
    pid_y_.initPid(p_y_, i_y_, d_y_, i_y_max_, i_y_min_);
    pid_theta_.initPid(p_theta_, i_theta_, d_theta_, i_theta_max_, i_theta_min_);
}

geometry_msgs::Twist PIDPush::getVelocities(){
    //initialize value
    geometry_msgs::Twist cmd = getNullTwist();

    //robot displacement from line object-target
    vec displacement_point_ = closestPointOnLine(pose_robot_.x, pose_robot_.y, pose_object_.pose.position.x, pose_object_.pose.position.y, current_target_.pose.position.x, current_target_.pose.position.y);

    geometry_msgs::PoseStamped pp = pose_object_;
    pp.pose.position.x = displacement_point_(0);
    pp.pose.position.y = displacement_point_(1);
    publishPoint(pp);

    cout << displacement_point_<<endl;
    vec robot_displacement_(2);
    robot_displacement_(0) = displacement_point_(0) - pose_robot_.x;
    robot_displacement_(1) = displacement_point_(1) - pose_robot_.y;

    // error object-target
    vec object_error_(2);
    object_error_(0) = current_target_.pose.position.x - pose_object_.pose.position.x;
    object_error_(1) = current_target_.pose.position.y - pose_object_.pose.position.y;

    // robot_error_vector = vector sum
    vec robot_error_(2);
    robot_error_(0) = robot_displacement_(0) +  object_error_(0);
    robot_error_(1) = robot_displacement_(1) +  object_error_(1);

    // transform to robot frame
    vec robot_error_R_ = rotate2DVector(robot_error_, -pose_robot_.theta);
    cout <<robot_error_R_ <<endl;

    cmd.linear.x = pid_x_.computeCommand(robot_error_R_(0), ros::Duration(time_step_));
    cmd.linear.y = pid_y_.computeCommand(robot_error_R_(1), ros::Duration(time_step_));


    //    //orientation error
    //    double err_th = aR2P - pose_robot_.theta;
    double err_th = aO2P - pose_robot_.theta;
    cmd.angular.z =  pid_theta_.computeCommand(err_th, ros::Duration(time_step_));

    cout << cmd<<endl;
    return cmd;

}



