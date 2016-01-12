#ifndef PUSHPLANNER_H
#define PUSHPLANNER_H

#include <string>
#include <limits>
#include <math.h>
#include <ros/ros.h>
#include <armadillo>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <squirrel_object_manipulation/math_utils.hpp>
#include <squirrel_object_manipulation/conversion_utils.hpp>

using namespace ros;
using namespace std;

enum PushState {
    INACTIVE,
    PUSH,
    // APPROACH,
    RELOCATE
    // ABBORTING
};

enum ObjectState {
    INACTION,
    DETACHED,
    SUCCESS,
    LOST
};

class PushPlanner{

private:

    string global_frame_;
    string local_frame_;
    double lookahead_;
    double goal_toll_;

    bool visualise_;
    bool state_machine_;
    bool rel_;

    ros::Publisher vis_points_pub_;
    ros::Publisher marker_target_c_;
    ros::Publisher marker_object_c_;
    ros::Publisher marker_point_;
    void publishWaypointMarkerArray(ros::NodeHandle nh);

    

protected:

    PushState push_state_;
    ros::NodeHandle  private_nh;

    double err_t_toll_;
    double err_th_toll_;
    double vel_lin_max_;
    double vel_ang_max_;
    double vel_x_max_, vel_y_max_;
    double controller_frequency_, time_step_;

    arma::vec relocate_target_;
    double relocate_target_orient_;
    double error_orient_sgn_ ;

    geometry_msgs::Pose2D pose_robot_;
    geometry_msgs::PoseStamped pose_object_;
    geometry_msgs::PoseStamped goal_;
    nav_msgs::Path pushing_path_;
    geometry_msgs::PoseStamped current_target_;

    geometry_msgs::PoseStamped getLookaheadPoint();

    ros::NodeHandle nh;

    virtual void initChild() = 0;

public:

    bool goal_reached_;
    bool push_active_;

    PushPlanner();
    PushPlanner(string local_frame_, string global_frame_, geometry_msgs::Pose2D pose_robot_, geometry_msgs::PoseStamped pose_object_, nav_msgs::Path pushing_path_, double lookahead_,  double goal_toll_, bool state_machine_, double controller_frequency_);
    void initialize(string local_frame_, string global_frame_, geometry_msgs::Pose2D pose_robot_, geometry_msgs::PoseStamped pose_object_, nav_msgs::Path pushing_path_, double lookahead_, double goal_toll_, bool state_machine_, double controller_frequency_);

    virtual void updatePushPlanner(geometry_msgs::Pose2D pose_robot_, geometry_msgs::PoseStamped pose_object_);
    virtual geometry_msgs::Twist getVelocities() = 0;

    geometry_msgs::Twist getControlCommand();
    void setLookahedDistance(double d);

    geometry_msgs::Twist relocateVelocities();

    void visualisationOn();
    void visualisationOff();
    void publishMarkerTargetCurrent(geometry_msgs::PoseStamped t_pose);
    void publishMarkerObjectCurrent(geometry_msgs::PoseStamped t_pose);
    void publishPoint(geometry_msgs::PoseStamped t_pose);


    void startPush();
    void stopPush();

};

#endif // PUSHPLANNER_H
