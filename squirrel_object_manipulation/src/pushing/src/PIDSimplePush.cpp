#include "../include/PIDSimplePush.hpp"

using namespace std;

PIDSimplePush::PIDSimplePush():
    PushPlanner()
{
    private_nh.param("push/error_theta_tolerance", err_th_toll_,0.01);
    private_nh.param("push/error_target_tolerance", err_t_toll_ ,0.01);
    private_nh.param("push/velocity_angular_max", vel_ang_max_ , 0.6);
    private_nh.param("push/velocity_linear_max", vel_lin_max_ , 0.8);
    private_nh.param("push/velocity_y_max", vel_y_max_ , 0.8);

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

void PIDSimplePush::initChild() {

    pid_x_.initPid(p_x_, i_x_, d_x_, i_x_max_, i_x_min_);
    pid_y_.initPid(p_y_, i_y_, d_y_, i_y_max_, i_y_min_);
    pid_theta_.initPid(p_theta_, i_theta_, d_theta_, i_theta_max_, i_theta_min_);
}

geometry_msgs::Twist PIDSimplePush::getVelocities(){
    //initialize value
    geometry_msgs::Twist cmd = getNullTwist();

    //translation error object-target
    double dO2T = distancePoints(pose_object_.pose.position.x, pose_object_.pose.position.y, current_target_.pose.position.x, current_target_.pose.position.y);
    cmd.linear.x = pid_x_.computeCommand(dO2T, ros::Duration(time_step_));

    //angle object-robot-target
    double aORT =  angle3Points(pose_object_.pose.position.x, pose_object_.pose.position.y, pose_robot_.x, pose_robot_.y, current_target_.pose.position.x, current_target_.pose.position.y);
    //distance robot to the line object-target
    double dRlOT = distance2Line(pose_robot_.x, pose_robot_.y, pose_object_.pose.position.x, pose_object_.pose.position.y, current_target_.pose.position.x, current_target_.pose.position.y);
    if (aORT > 0) {
        cmd.linear.y = pid_y_.computeCommand(dRlOT, ros::Duration(time_step_));
    }
    else {
        cmd.linear.y = pid_y_.computeCommand(-dRlOT, ros::Duration(time_step_));
    }

    //angle between robot pose and target
    double aR2P = atan2(current_target_.pose.position.y - pose_robot_.y, current_target_.pose.position.x - pose_robot_.x);
    if (isnan(aR2P)) aR2P = 0;

    //orientation error
    double err_th = aR2P - pose_robot_.theta;
    cmd.angular.z =  pid_theta_.computeCommand(err_th, ros::Duration(time_step_));

    return cmd;

}
