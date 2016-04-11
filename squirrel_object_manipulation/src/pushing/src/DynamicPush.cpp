#include "../include/DynamicPush.hpp"


using namespace std;
using namespace arma;

DynamicPush::DynamicPush():
    PushPlanner()
{
    private_nh.param("push/velocity_angular_max", vel_ang_max_ , 0.8);
    private_nh.param("push/velocity_linear_max", vel_lin_max_ , 0.2);
    private_nh.param("push/velocity_linear_min", vel_lin_min_ , 0.05);

    private_nh.param("push/proportional_theta", p_theta_, 1.6);
    private_nh.param("push/derivative_theta", d_theta_, 0.4);
    private_nh.param("push/integral_theta", i_theta_, 0.0);
    private_nh.param("push/integral_theta_max", i_theta_max_, 0.8);
    private_nh.param("push/integral_theta_min", i_theta_min_, -0.8);

    velocity_pub_ = nh.advertise<std_msgs::Float64>("/push_action/velocity", 1000);

}

void DynamicPush::initChild() {
    

    pid_theta_.initPid(p_theta_, i_theta_, d_theta_, i_theta_max_, i_theta_min_);

    mi_alpha= 0.0;
    sigma_alpha = 0;

    mi_theta = M_PI;
    sigma_theta = M_PI / 3;
    //sigma_theta = 1.0;
    count_dr = 100;

    aPORp = aPOR;
    count_all = 1;
    
    theta_vec.resize(1);
    alpha_vec.resize(1);

    data_cont_mat_.set_size(pushing_path_.poses.size(), 11);

    
}

void DynamicPush::updateChild() {
    
    
    //the angle object-robot-target
    aPOR =  angle3Points(current_target_.pose.position.x, current_target_.pose.position.y, pose_object_.pose.position.x, pose_object_.pose.position.y, pose_robot_.x, pose_robot_.y);
    if(aPOR < 0) aPOR = 2 * M_PI + aPOR;
    theta_vec(theta_vec.n_elem - 1) = aPOR;
    theta_vec.resize(theta_vec.n_elem  + 1);


    if (distancePoints(pose_object_.pose.position.x, pose_object_.pose.position.y, previous_pose_object_.pose.position.x, previous_pose_object_.pose.position.y) > 0.00){

        expected_dir = getVectorAngle(previous_target_.pose.position.x - previous_pose_object_.pose.position.x, previous_target_.pose.position.y - previous_pose_object_.pose.position.y);
        executed_dir = getVectorAngle(pose_object_.pose.position.x - previous_pose_object_.pose.position.x, pose_object_.pose.position.y - previous_pose_object_.pose.position.y);
        result_dir = getVectorAngle(previous_target_.pose.position.x - pose_object_.pose.position.x, previous_target_.pose.position.y - pose_object_.pose.position.y);

        previous_pose_object_ = pose_object_;

        // cout<<"executed dir "<< executed_dir<<endl;

        count_all ++;

        double tempA = mi_alpha;
        mi_alpha = mi_alpha + (alpha - mi_alpha) / count_all;
        sigma_alpha = ((count_all - 1) * sigma_alpha + (alpha - tempA) * (alpha - mi_alpha)) / count_all;

        alpha = expected_dir - executed_dir;
        //alpha = expected_dir - result_dir;


        if(alpha - alpha_old < - 0.01){
            //cout<<" alpha_old "<<alpha_old<<" alpha "<<alpha<<" diff "<<alpha - alpha_old<<endl;

            count_dr ++;

            double tempT = mi_theta;
            mi_theta = mi_theta + (aPORp - mi_theta) / count_dr;
            sigma_theta = ((count_dr - 1) * sigma_theta + (aPORp - tempT)*(aPORp - mi_theta)) / count_dr;
            if (sigma_theta < M_PI / 6) {
                sigma_theta = M_PI / 3;
                //cout<<"update sigma_theta"<<endl;
            }
            if ((abs(mi_theta) < M_PI / 2) || (abs(mi_theta) > 3 * M_PI / 2)){
                mi_theta = M_PI;
                //cout<<"update mi_theta"<<endl;
            }
            //cout<<" sigma_theta "<<sigma_theta<<" mi_theta "<<mi_theta<<endl;
        }
        alpha_vec.resize(alpha_vec.n_elem  + 1);
        alpha_vec(alpha_vec.n_elem - 1) = alpha;

        alpha_old = alpha;

    }
    aPORp = aPOR;

    //cout<<"aPOR "<<aPOR<<" sigma_theta "<<sigma_theta<<" mi_theta "<<mi_theta<<endl;
    psi_push_ = getGaussianVal(aPOR, sigma_theta, mi_theta);
    psi_rel_ = 1 - psi_push_;
    //cout<<" psi_push "<<psi_push_<<"psi relocate "<<psi_rel_<<endl;
    if(abs(aPOR - M_PI) > 0.4){
            psi_push_  = 0;
        }


    //matrix update

    data_cont_mat_(elem_count_, 0) = current_time_;
    data_cont_mat_(elem_count_, 1) = aPOR;
    data_cont_mat_(elem_count_, 2) = psi_push_;
    data_cont_mat_(elem_count_, 3) = sigma_theta;
    data_cont_mat_(elem_count_, 4) = mi_theta;
    data_cont_mat_(elem_count_, 5) = cmd.linear.x;
    data_cont_mat_(elem_count_, 6) = cmd.linear.y;
    data_cont_mat_(elem_count_, 7) = cmd.angular.z;
    data_cont_mat_(elem_count_, 8) = expected_dir;
    data_cont_mat_(elem_count_, 9) = executed_dir;
    data_cont_mat_(elem_count_, 10) = alpha;
    if (elem_count_ == data_cont_mat_.n_rows - 1) data_cont_mat_.resize(data_cont_mat_.n_rows + pushing_path_.poses.size(), data_cont_mat_.n_cols);


}

void DynamicPush::saveDataChild(string path){

    std::ofstream rFile;

    string nameF = path + experimentName + "_cont_data.txt";
    rFile.open(nameF.c_str());

    for (int i = 0; i < elem_count_; i ++){
        for(int j = 0; j <data_cont_mat_.n_cols; j++){
            rFile << data_cont_mat_(i, j) << "\t";
        }
        rFile << endl;

    }
    rFile.close();
}

geometry_msgs::Twist DynamicPush::getVelocities(){
    //initialize value
    cmd = getNullTwist();

    double vx_push =  psi_push_ * sign(cos(aPOR)) * cos(aPOR);
    double vy_push =  psi_push_ * sign(cos(aPOR)) * sin(aPOR);

    double vx_relocate = - psi_rel_ * sign(sin(aPOR)) * sin(aPOR);
    double vy_relocate =  psi_rel_ * sign(sin(aPOR)) * cos(aPOR);

    double vx_compensate =  - psi_push_ * sin(alpha) * (mean(alpha_vec) - alpha);
    //double vy_compensate =  - psi_push_ * abs(sin(alpha)) * (mean(alpha_vec) - alpha);
    double vy_compensate =  - psi_push_ * cos(alpha) * (mean(alpha_vec) - alpha);


//    if (sqrt (vx_compensate * vx_compensate + vy_compensate * vy_compensate) > 0.6){
//        vx_compensate = 0;
//        vy_compensate = 0;
//    }

    double vx =  vx_push + vx_relocate + vx_compensate;
    double vy =  vy_push + vy_relocate + vy_compensate;

    // transform to robot frame
    vec v = rotate2DVector(vx, vy, rotationDifference(aO2P, pose_robot_.theta));

    double V = vel_lin_max_ / (1 +  abs(mi_alpha));
    if(V < vel_lin_min_) V = vel_lin_min_;


    std_msgs::Float64 vel_;
    vel_.data = V;
    velocity_pub_.publish(vel_);

    cmd.linear.x = V * v(0) / getNorm(v);
    cmd.linear.y = V * v(1) / getNorm(v);

    double orient_error = rotationDifference(aR2O,pose_robot_.theta);
    if(orient_error>0.3){
            cmd.linear.x = 0;
            cmd.linear.y = 0;

        }
    cmd.angular.z = pid_theta_.computeCommand(orient_error, ros::Duration(time_step_));


    return cmd;

}



