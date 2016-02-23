#include "../include/DynamicPush.hpp"

using namespace std;
using namespace arma;

DynamicPush::DynamicPush():
    PushPlanner()
{
    private_nh.param("push/error_theta_tolerance", err_th_toll_,0.01);
    private_nh.param("push/error_target_tolerance", err_t_toll_ ,0.01);
    private_nh.param("push/velocity_angular_max", vel_ang_max_ , 0.6);
    private_nh.param("push/velocity_linear_max", vel_lin_max_ , 0.3);
    private_nh.param("push/velocity_y_max", vel_y_max_ , 0.3);

    private_nh.param("push/proportional_x", p_x_, 1.0);
    private_nh.param("push/derivative_x", d_x_, 0.0);
    private_nh.param("push/integral_x", i_x_, 0.0);
    private_nh.param("push/integral_x_max", i_x_max_, 1.0);
    private_nh.param("push/integral_x_min", i_x_min_, -1.0);

    private_nh.param("push/proportional_y", p_y_, 1.0);
    private_nh.param("push/derivative_y", d_y_, 0.0);
    private_nh.param("push/integral_y", i_y_, 0.0);
    private_nh.param("push/integral_y_max", i_y_max_, 1.0);
    private_nh.param("push/integral_y_min", i_y_min_, -1.0);

    private_nh.param("push/proportional_theta", p_theta_, 1.0);
    private_nh.param("push/derivative_theta", d_theta_, 0.3);
    private_nh.param("push/integral_theta", i_theta_, 0.3);
    private_nh.param("push/integral_theta_max", i_theta_max_, 1.0);
    private_nh.param("push/integral_theta_min", i_theta_min_, -1.0);
}

void DynamicPush::initChild() {

    pid_x_.initPid(p_x_, i_x_, d_x_, i_x_max_, i_x_min_);
    pid_y_.initPid(p_y_, i_y_, d_y_, i_y_max_, i_y_min_);
//    pid_xd_.initPid(2 * p_x_, 2 * i_x_,2 * d_x_, i_x_max_, i_x_min_);
//    pid_yd_.initPid(2 * p_x_, 2 * i_x_,2 * d_x_, i_y_max_, i_y_min_);
//    pid_theta_.initPid(p_theta_, i_theta_, d_theta_, i_theta_max_, i_theta_min_);
    mi_theta_p_ = 3.14;
    sigma_theta_p_= 1.0;
    count = 1;
}

void DynamicPush::updateChild() {

    mi_theta_p_ = 3.14;
    sigma_theta_p_= 1.0;
}

geometry_msgs::Twist DynamicPush::getVelocities(){
    //initialize value
    geometry_msgs::Twist cmd = getNullTwist();

    //robot displacement from line object-target
 //   vec displacement_point_ = closestPointOnLine(pose_robot_.x, pose_robot_.y, pose_object_.pose.position.x, pose_object_.pose.position.y, current_target_.pose.position.x, current_target_.pose.position.y);

//    geometry_msgs::PoseStamped pp = pose_object_;
//    pp.pose.position.x = displacement_point_(0);
//    pp.pose.position.y = displacement_point_(1);
//    publishPoint(pp);

//    cout << displacement_point_<<endl;
//    vec robot_displacement_(2);
//    robot_displacement_(0) = displacement_point_(0) - pose_robot_.x;
//    robot_displacement_(1) = displacement_point_(1) - pose_robot_.y;

//    // transform to robot frame
//    vec robot_displacement_r = rotate2DVector(robot_displacement_, -pose_robot_.theta);

//    double e_dx, e_dy;
//    e_dx = pid_xd_.computeCommand(robot_displacement_r(0), ros::Duration(time_step_));
//    e_dy = pid_yd_.computeCommand(robot_displacement_r(1), ros::Duration(time_step_));


//    // error object-target
//    vec object_error_(2);
//    object_error_(0) = current_target_.pose.position.x - pose_object_.pose.position.x;
//    object_error_(1) = current_target_.pose.position.y - pose_object_.pose.position.y;

//    // transform to robot frame
//    vec object_error_r = rotate2DVector(object_error_, -pose_robot_.theta);

//    double e_ox, e_oy;
//    e_ox = pid_x_.computeCommand(object_error_r(0), ros::Duration(time_step_));
//    e_oy = pid_y_.computeCommand(object_error_r(1), ros::Duration(time_step_));

    //the angle object-robot-target
    double aPOR =  angle3Points(current_target_.pose.position.x, current_target_.pose.position.y, pose_object_.pose.position.x, pose_object_.pose.position.y, pose_robot_.x, pose_robot_.y);

    double gain = 0.3;



     double vx_push = - sign(cos(aPOR)) * pid_x_.computeCommand(-cos(aPOR), ros::Duration(time_step_));
     double vy_push = - sign(cos(aPOR)) * pid_y_.computeCommand(-sin(aPOR), ros::Duration(time_step_));

    // double vx_push = - cos(aPOR) * (-cos(aPOR));
    // double vy_push = - cos(aPOR) * (-sin(aPOR));

    double vx_relocate = -sin(aPOR);
    double vy_relocate = cos(aPOR);
   // cmd.linear.x = gain * cos(2 * aPOR);
   // cmd.linear.y = - gain * sin(2 * aPOR);

    cmd.linear.x = gain * ( vx_push + sin(aPOR) * vx_relocate);
    cmd.linear.y = gain * ( vy_push + sin(aPOR) * vy_relocate);
    cout<<aPOR<<" cos(aPOR) "<<cos(aPOR)<<" sin(aPOR) "<<sin(aPOR)<<endl;

//    // robot_error_vector = vector sum
//    vec robot_error_(2);
//    robot_error_(0) = robot_displacement_(0) +  object_error_(0);
//    robot_error_(1) = robot_displacement_(1) +  object_error_(1);


    //    //orientation error
    //    double err_th = aR2P - pose_robot_.theta;
//    double err_th = aO2P - pose_robot_.theta;
//    cmd.angular.z =  pid_theta_.computeCommand(err_th, ros::Duration(time_step_));

//    cmd.linear.x = gain_x * abs(e_ox + e_dx);
//    cmd.linear.y = gain_y * abs(e_oy + e_dy);


    return cmd;

}



