#ifndef DYNAMICPUSHORIENTVM_H
#define DYNAMICPUSHORIENTVM_H
#include "PushPlanner.hpp"

#include <control_toolbox/pid.h>
#include <std_msgs/Float64.h>

class DynamicPushOrientVM : public PushPlanner {

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
    arma::vec aPOR_vec, mi_alpha_vec, sigma_alpha_vec, aPORp_vec, mi_posterior_vec, sigma_posterior_vec, mi_posterior_pred_vec, sigma_posterior_pred_vec, mi_prior_vec, sigma_prior_vec, sum_alpha_vec;
    arma::vec model_orient_vec;
    double gamma, gamma_old, mi_gamma, sigma_gamma;
    double beta, beta_old, mi_beta, sigma_beta;
    int count_dr, count_all;
    arma::vec count_dr_vec, count_all_vec;
    double psi_push_, psi_rel_;
    double expected_dir, executed_dir, result_dir;

    arma::mat data_cont_mat_,param_VM_vec, w_mat, param_VM_mat;
    arma::vec alpha_vec, gamma_vec, param_VM;
    arma::vec psi_push_vec, alpha_g_vec, w_vec;

    arma::vec like_w_;

    int object_precision_;


protected:
    void initChild();
    void updateChild();

public:

    DynamicPushOrientVM();
    geometry_msgs::Twist getVelocities();
    void saveDataChild(string path);


};


#endif // DynamicPushOrientVM_H
