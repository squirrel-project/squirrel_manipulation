#ifndef ROBOTINOBASECONTROL
#define ROBOTINOBASECONTROL

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread.hpp>
#include <control_toolbox/pid.h>
#include <tf/tf.h>

class RobotinoBaseControl {

private:

	static auto constexpr ROBOTINO_MOVE_TOPIC = "/cmd_vel";
	static auto constexpr ROBOTINO_ODOM_TOPIC = "/odom";

    ros::NodeHandle  private_nh;

    ros::Subscriber subOdometry;
    ros::Publisher pubMove;

    nav_msgs::Odometry odometry;

    boost::thread* move_base_thread_;
    bool start_move_base_;

    double controller_frequency_, time_step_;
    double vel_ang_max_;
    double p_theta_, d_theta_, i_theta_, i_theta_min_, i_theta_max_;

    double desired_theta_;

    control_toolbox::Pid pid_theta_;
    geometry_msgs::Twist current_base_vel_;

    boost::mutex robot_pose_mutex_;
    boost::mutex move_pose_mutex_;

    void callbackOdometry(nav_msgs::Odometry msg);
    void moveBaseThread();

    geometry_msgs::Twist getNullTwist();
    double rotationDifference(double angle, double theta_robot);


public:

    RobotinoBaseControl(ros::NodeHandle& node, double  controller_frequency_, double max_ang_vel_);
    ~RobotinoBaseControl();

    void move(double desired_theta);

    double getCurrentState();

};

#endif
