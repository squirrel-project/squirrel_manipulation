#include "../include/DynamicPush.hpp"


using namespace std;
using namespace arma;

DynamicPush::DynamicPush():
    PushPlanner()
{
    private_nh.param("push/velocity_angular_max", vel_ang_max_ , 0.3);
    private_nh.param("push/velocity_linear_max", vel_lin_max_ , 0.1); //0.15
    private_nh.param("push/velocity_linear_min", vel_lin_min_ , 0.08); //0.08

    private_nh.param("push/proportional_alpha", p_alpha_, 0.6);
    private_nh.param("push/derivative_alpha", d_alpha_, 0.4);
    private_nh.param("push/integral_alpha", i_alpha_, 0.0);
    private_nh.param("push/integral_alpha_max", i_alpha_max_, 0.8);
    private_nh.param("push/integral_alpha_min", i_alpha_min_, -0.8);

    private_nh.param("push/proportional_alpha", p_x_, 0.6);
    private_nh.param("push/derivative_alpha", d_x_, 0.4);
    private_nh.param("push/integral_alpha", i_x_, 0.0);

    private_nh.param("push/proportional_alpha", p_y_, 0.6);
    private_nh.param("push/derivative_alpha", d_y_, 0.4);
    private_nh.param("push/integral_alpha", i_y_, 0.0);


    private_nh.param("push/publish_values", pub_values_, true);

    private_nh.param("push/proportional_ff", p_ff_, 0.02);
    private_nh.param("push/derivative_ff", d_ff_, 0.0);
    private_nh.param("push/integral_ff", i_ff_, 0.01);
    private_nh.param("push/integral_ff", i_ff_max_, 0.1);
    private_nh.param("push/integral_ff", i_ff_min_, -0.1);




    velocity_pub_ = nh.advertise<std_msgs::Float64>("/push_action/velocity", 100);
    psi_push_pub_ = nh.advertise<std_msgs::Float64>("/push_action/psi_push", 100);
    psi_rel_pub_ = nh.advertise<std_msgs::Float64>("/push_action/psi_relocate", 100);
    sigma_alpha_pub_ = nh.advertise<std_msgs::Float64>("/push_action/sigma_gamma", 100);
    mi_alpha_pub_ = nh.advertise<std_msgs::Float64>("/push_action/mi_gamma", 100);
    compensate_pub_ = nh.advertise<geometry_msgs::Pose2D>("/push_action/compensate_direction", 100);
    push_pub_ = nh.advertise<geometry_msgs::Pose2D>("/push_action/push_direction", 100);
    rel_pub_ = nh.advertise<geometry_msgs::Pose2D>("/push_action/relocate_direction", 100);
    compensate_pub_ = nh.advertise<geometry_msgs::Pose2D>("/push_action/compensate_direction", 100);
}

void DynamicPush::initChild() {
    

    pid_alpha_.initPid(p_alpha_, i_alpha_, d_alpha_, i_alpha_max_, i_alpha_min_);
    pid_ff_.initPid(p_ff_, i_ff_, d_ff_, i_ff_max_, i_ff_min_);

    mi_gamma= 0.0;
    sigma_gamma = 0;

    mi_alpha = M_PI;
    sigma_alpha = M_PI /4;
    mi_posterior = M_PI;
    sigma_posterior = M_PI /4;
    mi_prior = M_PI;
    sigma_prior = M_PI / 4;
    mi_posterior_pred = M_PI;
    sigma_posterior_pred = M_PI / 4;
    //sigma_alpha = 1.0;
    count_dr = 100;
    sum_alpha = count_dr * mi_alpha;

    aPORp = aPOR;
    count_all = 1;
    
    alpha_vec.resize(1);
    gamma_vec.resize(1);


    if(sim_) vel_lin_max_ = 0.3;

    data_cont_mat_.set_size(pushing_path_.poses.size(), 11);
}

void DynamicPush::updateChild() {
    
    
    //the angle object-robot-target
    aPOR =  angle3Points(current_target_.pose.position.x, current_target_.pose.position.y, pose_object_.pose.position.x, pose_object_.pose.position.y, pose_robot_.x, pose_robot_.y);
    if(aPOR < 0) aPOR = 2 * M_PI + aPOR;
    alpha_vec(alpha_vec.n_elem - 1) = aPOR;
    alpha_vec.resize(alpha_vec.n_elem  + 1);

    //update only if there was change in the object movement
    if (distancePoints(pose_object_.pose.position.x, pose_object_.pose.position.y, previous_pose_object_.pose.position.x, previous_pose_object_.pose.position.y) > 0.00){

        expected_dir = getVectorAngle(previous_target_.pose.position.x - previous_pose_object_.pose.position.x, previous_target_.pose.position.y - previous_pose_object_.pose.position.y);
        executed_dir = getVectorAngle(pose_object_.pose.position.x - previous_pose_object_.pose.position.x, pose_object_.pose.position.y - previous_pose_object_.pose.position.y);

        previous_pose_object_ = pose_object_;

        count_all ++;

        double tempA = mi_gamma;
        mi_gamma = mi_gamma + (gamma - mi_gamma) / count_all;
        sigma_gamma = ((count_all - 1) * sigma_gamma + (gamma - tempA) * (gamma - mi_gamma)) / count_all;

        gamma = expected_dir - executed_dir;
        if(gamma > 2*M_PI) gamma = gamma - 2*M_PI;
        if(gamma < -2*M_PI) gamma = gamma + 2*M_PI;

        //cout<<" gamma_old "<<gamma_old<<" gamma "<<gamma<<" diff "<<gamma - gamma_old<<endl;

        if(((abs(gamma) - abs(gamma_old))< -0.01)){

            //cout<<"here"<<endl;
            count_dr ++;

            double tempT = mi_alpha;
            mi_alpha = mi_alpha + (aPORp - mi_alpha) / count_dr;
            sigma_alpha = ((count_dr - 1) * sigma_alpha + (aPORp - tempT)*(aPORp - mi_alpha)) / count_dr;
            sum_alpha = sum_alpha + aPORp;
            mi_prior = mi_posterior;
            sigma_prior = sigma_posterior;
            mi_posterior = (sigma_alpha * mi_prior/ count_dr+ sigma_prior*sum_alpha/count_dr)/(sigma_alpha/count_dr + sigma_prior);
            sigma_posterior = 1/(count_dr/sigma_alpha + 1/sigma_prior);
            mi_posterior_pred = mi_posterior;
            sigma_posterior_pred = sigma_posterior + sigma_alpha;
        }
        gamma_vec.resize(gamma_vec.n_elem  + 1);
        gamma_vec(gamma_vec.n_elem - 1) = gamma;

        gamma_old = gamma;

    }
    aPORp = aPOR;

    //cout<<"alpha sigma: "<<sigma_alpha<<" mi_alpha "<<mi_alpha<<" sig post "<<sigma_posterior<<"mi post "<<mi_posterior<<" sig post  pred"<<sigma_posterior_pred<<"mi post pred"<<mi_posterior_pred<<endl;
    psi_push_ = getGaussianVal(aPOR, sigma_posterior_pred, mi_posterior_pred);
    if (psi_push_ > abs(cos(aPOR))){
        //psi_push_ = abs(cos(aPOR));
       // cout << "filter psi "<< psi_push_<<" cos "<<cos(aPOR)<< endl;
        psi_push_ = abs(cos(aPOR));
    }
    psi_rel_ = sqrt(1 - psi_push_*psi_push_);
    filt_com = 1.0;
    if(dR2O >robot_diameter_/2 + object_diameter_/2+ 0.10){
        psi_rel_ = 0;
        filt_com = 0;
        psi_push_ = 1;
        cout<<"(push dynamic) dR2O >robot_diameter_ n"<<endl;
    }
    //    else if((abs(aPOR - M_PI) > 0.6)&& (cos(aPOR)<0)){
    //        psi_push_  = - psi_push_;
    //        cout<<"(push dynamic) alpha"<<endl;
    //    }
    else if( (cos(aPOR)>0)){
        psi_push_  = - psi_push_;
        //cout<<"(push dynamic) alpha"<<endl;
    }
    else if (sim_ && (dR2O >robot_diameter_/2 + object_diameter_/2+ 0.05)){
        psi_rel_ = 0;
        filt_com = 0;
        psi_push_ = 1;
        //cout<<"(push dynamic) dR2O >robot_diameter_ n"<<endl;

    }
    else if (sim_ && (abs(aPOR - M_PI) > 0.4)&& (cos(aPOR)<0)) { //0.3 bilo
        psi_push_ = 0;
        //cout<<"(push dynamic) alpha big"<<endl;
    }


    //matrix update

    data_cont_mat_(elem_count_, 0) = current_time_;
    data_cont_mat_(elem_count_, 1) = aPOR;
    data_cont_mat_(elem_count_, 2) = psi_push_;
    data_cont_mat_(elem_count_, 3) = sigma_alpha;
    data_cont_mat_(elem_count_, 4) = mi_alpha;
    data_cont_mat_(elem_count_, 5) = cmd.linear.x;
    data_cont_mat_(elem_count_, 6) = cmd.linear.y;
    data_cont_mat_(elem_count_, 7) = cmd.angular.z;
    data_cont_mat_(elem_count_, 8) = expected_dir;
    data_cont_mat_(elem_count_, 9) = executed_dir;
    data_cont_mat_(elem_count_, 10) = gamma;
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
    double vx_push, vy_push, vx_relocate, vy_relocate, V;

    if(!sim_){
        // real robot
//        double K1 = 0.05; double K2 = 0.1;
//        aPOR = aPOR -filt_com*K1*mi_gamma -  filt_com*K2*gamma;

        //double theta_v = getVectorAngle(sign(cos(aPOR)) * cos(aPOR),sign(cos(aPOR)) * sin(aPOR));
        double theta_v = getVectorAngle(cos(aPOR), sin(aPOR));

        theta_v = aPOR;

        vx_push = psi_push_  * sign(cos(theta_v)) * cos(theta_v);
        vy_push = psi_push_  * sign(cos(theta_v)) * sin(theta_v);


        vx_relocate = - psi_rel_ * sign(sin(mi_alpha - theta_v)) * sin(theta_v);
        vy_relocate =  psi_rel_ * sign(sin(mi_alpha - theta_v)) * cos(theta_v);

        vx_relocate = - psi_rel_ * sign(sin(theta_v)) * sin(theta_v);
        vy_relocate =  psi_rel_ * sign(sin(theta_v)) * cos(theta_v);


        double vx =  vx_push + vx_relocate ;
        double vy =  vy_push + vy_relocate ;

        theta_v = getVectorAngle(vx, vy);
        double K1 = 0.05; double K2 = 0.1;
        theta_v = theta_v-filt_com*K1*mi_gamma -  filt_com*K2*gamma;

        vec v = rotate2DVector(cos(theta_v), sin(theta_v), rotationDifference(aO2P, pose_robot_.theta));

        //vec v = rotate2DVector(vx, vy, rotationDifference(aO2P, pose_robot_.theta));

        V = vel_lin_max_ / (1 +  abs(mi_gamma));
        if(V < vel_lin_min_) V = vel_lin_min_;

        double rx = V * v(0) / getNorm(v);
        double ry = V * v(1) / getNorm(v);

        double orient_error = rotationDifference(aR2O,pose_robot_.theta);
        if(orient_error > 0.3){
            cmd.linear.x = 0;
            cmd.linear.y = 0;

            rx = 0;
            ry = 0;

        }
        cmd.angular.z = pid_alpha_.computeCommand(orient_error, ros::Duration(time_step_));


        cmd.linear.x = rx;
        cmd.linear.y = ry;


    }
    else{

        //simulation

        double theta_v = getVectorAngle(sign(cos(aPOR)) * cos(aPOR),sign(cos(aPOR)) * sin(aPOR));

        theta_v = theta_v - sign(sin(aPOR)) * (M_PI /2 + M_PI /5); //M_PI/3 bilo

        vx_push = psi_push_ * sign(cos(aPOR)) * cos(aPOR);
        vy_push = psi_push_  * sign(cos(aPOR)) * sin(aPOR);


        vx_relocate =  psi_rel_ * cos(theta_v);
        vy_relocate =  psi_rel_ * sin(theta_v);


        double vx =  vx_push + vx_relocate ;
        double vy =  vy_push + vy_relocate ;

        vec v = rotate2DVector(vx, vy, rotationDifference(aO2P, pose_robot_.theta));

        double rx = vel_lin_max_ * v(0) / getNorm(v);
        double ry = vel_lin_max_ * v(1) / getNorm(v);

        double orient_error = rotationDifference(aR2O,pose_robot_.theta);

        cmd.angular.z = pid_alpha_.computeCommand(orient_error, ros::Duration(time_step_));
        cmd.linear.x = rx;
        cmd.linear.y = ry;

    }

    if(pub_values_){

        geometry_msgs::Pose2D pub_values;

        pub_values.x = vx_push;
        pub_values.y = vy_push;
        push_pub_.publish(pub_values);

        pub_values.x = vx_relocate;
        pub_values.y = vy_relocate;
        rel_pub_.publish(pub_values);

        std_msgs::Float64 vel_;
        vel_.data = V;
        velocity_pub_.publish(vel_);

        vel_.data = psi_push_;
        psi_push_pub_.publish(vel_);

        vel_.data = psi_rel_;
        psi_rel_pub_.publish(vel_);

        vel_.data = sigma_alpha;
        sigma_alpha_pub_.publish(vel_);

        vel_.data = mi_alpha;
        mi_alpha_pub_.publish(vel_);

    }

    return cmd;

}



