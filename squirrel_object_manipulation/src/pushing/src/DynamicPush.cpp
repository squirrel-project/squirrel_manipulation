#include "../include/DynamicPush.hpp"
#include "gp.h"
#include "gp_utils.h"

//#include <Eigen/Dense>

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
    
    //pid_x_.initPid(p_x_, i_x_, d_x_, i_x_max_, i_x_min_);
    
    pid_x_.initDynamicReconfig(private_nh);
    pid_x_.setGains(p_x_, i_x_, d_x_, i_x_max_, i_x_min_);
    pid_y_.initDynamicReconfig(private_nh);
    pid_y_.setGains(p_y_, i_y_, d_y_, i_y_max_, i_y_min_);
    //    pid_xd_.initPid(2 * p_x_, 2 * i_x_,2 * d_x_, i_x_max_, i_x_min_);
    //    pid_yd_.initPid(2 * p_x_, 2 * i_x_,2 * d_x_, i_y_max_, i_y_min_);
    pid_theta_.initPid(p_theta_, i_theta_, d_theta_, i_theta_max_, i_theta_min_);
    
    mi_dr = 0.0;
    sigma_dr = 2 * object_diameter_;
    mi_theta = M_PI;
    
    sigma_theta= 1.0;
    count_dr = 100;
    dO2Pp = dO2P;
    dRlOTp = dRlOT;
    aPORp = aPOR;
    betap = M_PI / 2;
    
    sigma_e = 0.5;
    ep = 1;
    mi_e = 0.5;
    
    sigma_theta_all = 1.0;
    mi_theta_all = M_PI;
    thetap = aPOR;
    
    count_all = 0;
    
    theta_vec.resize(1);
    vx_p_.resize(1);
    vy_p_.resize(1);
    alpha_vec.resize(1);
    vdO2P_.resize(1);
    
    
    
}

void DynamicPush::updateChild() {
    
    
    //the angle object-robot-target
    aPOR =  angle3Points(current_target_.pose.position.x, current_target_.pose.position.y, pose_object_.pose.position.x, pose_object_.pose.position.y, pose_robot_.x, pose_robot_.y);
    
    if(aPOR < 0) aPOR = 2 * M_PI + aPOR;
    theta_vec(theta_vec.n_elem - 1) = aPOR;
    
    theta_vec.resize(theta_vec.n_elem  + 1);
    
    vx_p_(vx_p_.n_elem - 1) = -cos(aPOR);
    vy_p_(vy_p_.n_elem - 1) = -sin(aPOR);
    
    vx_p_.resize(vx_p_.n_elem + 1);
    vy_p_.resize(vy_p_.n_elem + 1);
    
    double expected_dir = getVectorAngle(previous_target_.pose.position.x - previous_pose_object_.pose.position.x, previous_target_.pose.position.y - previous_pose_object_.pose.position.y);
    double executed_dir = getVectorAngle(pose_object_.pose.position.x - previous_pose_object_.pose.position.x, pose_object_.pose.position.y - previous_pose_object_.pose.position.y);
    
    alpha_old = alpha;
    alpha = expected_dir - executed_dir;
    alpha_vec(alpha_vec.n_elem - 1) = alpha;
    alpha_vec.resize(alpha_vec.n_elem  + 1);
    
    beta = aO2P - tf::getYaw(pose_object_.pose.orientation);
    
    
    //distance object target point
    double dO2Ppn = distancePoints(pose_object_.pose.position.x, pose_object_.pose.position.y, previous_target_.pose.position.x, previous_target_.pose.position.y);
    
    vdO2P_(vdO2P_.n_elem - 1) = dO2Ppn;
    vdO2P_.resize(vdO2P_.n_elem + 1);
    double e_c = 0.0;
    
    //if(dO2Ppn < dO2Pp){
    if(abs(alpha)< abs(alpha_old)){
        
        count_dr ++;
        double tempD = mi_dr;
        mi_dr = mi_dr + (dRlOTp - mi_dr)/count_dr;
        sigma_dr = ((count_dr -1) * sigma_dr + (dRlOTp - tempD)*(dRlOTp - mi_dr )) / count_dr;
        
        double tempT = mi_theta;
        mi_theta = mi_theta + (aPORp - mi_theta)/count_dr;
        sigma_theta = ((count_dr -1) * sigma_theta + (aPORp - tempT)*(aPORp - mi_theta)) / count_dr;
        
        double tempB = mi_beta;
        mi_beta = mi_beta + (betap - mi_beta)/count_dr;
        sigma_beta = ((count_dr -1) * sigma_beta + (betap - tempB)*(betap - mi_beta)) / count_dr;
        
        e_c = 1.0;
    }
    
    
    dO2Pp = dO2Ppn;
    dRlOTp = dRlOT;
    aPORp = aPOR;
    betap = beta;
    
    count_all ++;
    double tempE = mi_e;
    mi_e = mi_e + (e_c - mi_e)/count_all;
    sigma_e = ((count_all -1) * sigma_e + (ep - tempE)*(ep - mi_e )) / count_dr;
    
    
    
    // psi_push_ = getGaussianVal(dRlOT, sigma_dr, mi_dr);
    // psi_rel_ = 1 - psi_push_;
    psi_rel_ = 1 - getGaussianVal(aPOR, sigma_theta, mi_theta);
    psi_push_ = getGaussianVal(aPOR, sigma_theta, mi_theta);
    
    if(abs(aPOR - M_PI) > 0.4){
        psi_push_  = 0;
    }

    
    
    
}

geometry_msgs::Twist DynamicPush::getVelocities(){
    //initialize value
    geometry_msgs::Twist cmd = getNullTwist();
    
    
    //pid_x_.setGains(p_x_, i_x_, abs(cos(aPOR)/2), i_x_max_, i_x_min_);
    //pid_y_.setGains(p_y_, i_y_, abs(cos(aPOR)/2), i_y_max_, i_y_min_);
    
    pid_x_.setGains(p_x_, i_x_, var(vx_p_), i_x_max_, i_x_min_);
    pid_y_.setGains(p_y_, i_y_, var(vy_p_), i_y_max_, i_y_min_);
    
    //double vx_push = - psi_push_ * sign(cos(aPOR)) * pid_x_.computeCommand(-cos(aPOR), ros::Duration(time_step_));
    //double vy_push = - psi_push_ * sign(cos(aPOR)) * pid_y_.computeCommand(-sin(aPOR), ros::Duration(time_step_));
    
    double vx_push =  psi_push_ * sign(cos(aPOR)) * cos(aPOR);
    double vy_push = psi_push_ * sign(cos(aPOR)) * sin(aPOR);
    
    double vx_relocate = - psi_rel_ * sign(sin(aPOR)) * sin(aPOR);
    double vy_relocate =  psi_rel_ * sign(sin(aPOR)) * cos(aPOR);
    
    //    double vx_compensate =  psi_push_*  cos(aPOR) * var(alpha_vec)  * cos(alpha);
    double vy_compensate =  psi_push_ * var(alpha_vec)  * sin(alpha);
    double vx_compensate = -  abs(psi_push_ * var(alpha_vec)*cos(alpha));
    vx_compensate = 0;
    vy_compensate = 0;
    
    cout<<"vx_compensate "<<vx_compensate<<endl;
    cout<<"vy_compensate "<<vy_compensate<<endl;
    
    double vx_stabilize =  psi_push_ * cos(aPOR) * 0.4 *(mi_beta - beta);
    
    //    double vx_push = - cos(aPOR) * (-cos(aPOR));
    //    double vy_push = - cos(aPOR) * (-sin(aPOR));
    
    //    double vx_relocate = -sin(aPOR) * sin(aPOR);
    //    double vy_relocate = sin(aPOR) * cos(aPOR);
    //double vx =  gain * cos(2 * aPOR);
    //double vy =  gain * sin(2 * aPOR);
    
    double vx =  vx_push +  vx_relocate + vx_compensate;
    double vy =  vy_push + vy_relocate + vy_compensate;
    
    // transform to robot frame
    vec v = rotate2DVector(vx, vy, rotationDifference( aO2P, pose_robot_.theta));
    
    cout<<"vx "<<vx<<" vy "<<vy<<endl;
    cout <<"psi push "<< psi_push_<<" dRlOT "<<dRlOT<<" "<<sigma_dr<<" "<<mi_dr<<""<<endl;
    cout <<"psi rel "<< psi_rel_<<" aROP "<<aPOR<<" "<<sigma_theta<<" "<<mi_theta<<""<<endl<<endl;
    cout <<"count "<< count_dr<<endl;
    cout<<v<<endl;
    
    //cmd.linear.x = gain * ( vx_push + sin(aPOR) * vx_relocate);
    // cmd.linear.y = gain * ( vy_push + sin(aPOR) * vy_relocate);
    
    
    cmd.linear.x = vel_lin_max_ * v(0) / getNorm(v);
    cmd.linear.y = vel_lin_max_ * v(1) / getNorm(v);

    cmd.angular.z = pid_theta_.computeCommand(rotationDifference(aR2O,pose_robot_.theta), ros::Duration(time_step_));
    
    return cmd;
    
}



