#include "squirrel_object_manipulation/RobotinoBaseControl.hpp"

using namespace ros;
using namespace std;

RobotinoBaseControl::RobotinoBaseControl(ros::NodeHandle& node, double controller_frequency_, double vel_ang_max_): controller_frequency_(controller_frequency_), vel_ang_max_(vel_ang_max_), start_move_base_(false), private_nh("~") {

    private_nh.param("baseControl/proportional_theta", p_theta_, 0.6);
    private_nh.param("baseControl/derivative_theta", d_theta_, 0.4);
    private_nh.param("baseControl/integral_theta", i_theta_, 0.0);
    private_nh.param("baseControl/integral_theta_max", i_theta_max_, 0.8);
    private_nh.param("baseControl/integral_theta_min", i_theta_min_, -0.8);

    this->time_step_ = 1 / controller_frequency_;

    subOdometry = node.subscribe(ROBOTINO_ODOM_TOPIC, 1, &RobotinoBaseControl::callbackOdometry, this);
    pubMove = node.advertise<geometry_msgs::Twist>(ROBOTINO_MOVE_TOPIC, 1);

    move_base_thread_ = new boost::thread(boost::bind(&RobotinoBaseControl::moveBaseThread, this));

    pid_theta_.initPid(p_theta_, i_theta_, d_theta_, i_theta_max_, i_theta_min_);

    usleep(1e6);

}

RobotinoBaseControl::~RobotinoBaseControl(){

    move_base_thread_->interrupt();
    move_base_thread_->join();

    delete move_base_thread_;
}


void RobotinoBaseControl::callbackOdometry(nav_msgs::Odometry msg) {
    robot_pose_mutex_.lock();
    odometry = msg;
    robot_pose_mutex_.unlock();
}

void RobotinoBaseControl::move(double desired_theta){

    move_pose_mutex_.lock();
    desired_theta_ = desired_theta;
    start_move_base_ = true;
    move_pose_mutex_.unlock();

}

void RobotinoBaseControl::moveBaseThread(){

    ros::Rate moveBaseRate(controller_frequency_);
    ros::spinOnce();

    while (ros::ok){

        if(start_move_base_) {

            current_base_vel_ = getNullTwist();
            double orient_error = rotationDifference(desired_theta_, tf::getYaw(odometry.pose.pose.orientation));
            current_base_vel_.angular.z = pid_theta_.computeCommand(orient_error, ros::Duration(time_step_));
            if(fabs(current_base_vel_.angular.z) > vel_ang_max_) current_base_vel_.angular.z = (current_base_vel_.angular.z > 0 ? vel_ang_max_ : - vel_ang_max_);
            pubMove.publish(current_base_vel_);
            start_move_base_ = false;
        }

        moveBaseRate.sleep();
        ros::spinOnce();
    }

}

geometry_msgs::Twist RobotinoBaseControl::getNullTwist(){

    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;

    return cmd;
}

double RobotinoBaseControl::rotationDifference(double angle, double theta_robot){

    double err_th = angle - theta_robot;

    if(err_th > M_PI) err_th = - (2 * M_PI - err_th);
    if(err_th < -M_PI) err_th = 2 * M_PI + err_th;

    return err_th;
}

double RobotinoBaseControl::getCurrentState(){

    return tf::getYaw(odometry.pose.pose.orientation);
}





