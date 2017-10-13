#ifndef DYNAMICPUSHVM_H
#define DYNAMICPUSHVM_H

#include "PushPlanner.hpp"

#include <control_toolbox/pid.h>
#include <std_msgs/Float64.h>

class DynamicPushVM : public PushPlanner {

private:

    bool pub_values_;


    ros::Publisher velocity_pub_;
    ros::Publisher psi_push_pub_;
    ros::Publisher psi_rel_pub_;
    ros::Publisher compensate_pub_;
    ros::Publisher push_pub_;
    ros::Publisher rel_pub_;
    ros::Publisher sigma_alpha_pub_;
    ros::Publisher mi_alpha_pub_;

    control_toolbox::Pid pid_alpha_;
    control_toolbox::Pid pid_x_;
    control_toolbox::Pid pid_y_;
    control_toolbox::Pid pid_ff_;
    geometry_msgs::Twist cmd;
    double p_alpha_, d_alpha_, i_alpha_, i_alpha_min_, i_alpha_max_;
    double p_x_, d_x_, i_x_, i_x_min_, i_x_max_;
    double p_y_, d_y_, i_y_, i_y_min_, i_y_max_;
    double p_ff_, d_ff_, i_ff_, i_ff_min_, i_ff_max_;

    double filt_com;
    double aPOR, mi_alpha, sigma_alpha, aPORp, mi_posterior, sigma_posterior,mi_posterior_pred, sigma_posterior_pred, mi_prior, sigma_prior, sum_alpha;
    double gamma, gamma_old, mi_gamma, sigma_gamma;
    double beta, beta_old, mi_beta, sigma_beta;
    int count_dr, count_all;
    double psi_push_, psi_rel_;
    double expected_dir, executed_dir, result_dir;

    arma::mat data_cont_mat_;
    arma::vec alpha_vec, gamma_vec, w_vec, param_VM;



protected:
    void initChild();
    void updateChild();

public:

    DynamicPushVM();
    geometry_msgs::Twist getVelocities();
    void saveDataChild(string path);


};


#endif // DynamicPushVM_H
