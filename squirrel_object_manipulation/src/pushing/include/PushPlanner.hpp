#ifndef PUSHPLANNER_H
#define PUSHPLANNER_H

#include <string>
#include <limits>
#include <math.h>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <squirrel_object_manipulation/math_utils.hpp>


using namespace ros;
using namespace std;

class PushPlanner
{
private:
    string global_frame_;
    string local_frame_;
    double lookahead_;

    bool visualise_;
    ros::Publisher vis_points_pub_;
    ros::Publisher marker_target_c_;
    void publishWaypointMarkerArray(ros::NodeHandle nh);

    

protected:
    double err_t_toll_;
    double err_th_toll_;
    double vel_lin_;
    double vel_ang_;
    
    ros::NodeHandle private_nh;
    geometry_msgs::Pose2D pose_robot_;
    geometry_msgs::PoseStamped pose_object_;
    nav_msgs::Path pushing_path_;
    geometry_msgs::PoseStamped getLookaheadPoint();

    ros::NodeHandle nh;

public:

    bool goal_reached_;

    PushPlanner();
    PushPlanner(string local_frame_, string global_frame_, geometry_msgs::Pose2D pose_robot_, geometry_msgs::PoseStamped pose_object_, nav_msgs::Path pushing_path_, double lookahead_);
    void initialize(string local_frame_, string global_frame_, geometry_msgs::Pose2D pose_robot_, geometry_msgs::PoseStamped pose_object_, nav_msgs::Path pushing_path_, double lookahead_);
    virtual void updatePushPlanner(geometry_msgs::Pose2D pose_robot_, geometry_msgs::PoseStamped pose_object_) = 0;
    virtual geometry_msgs::Twist getVelocities() = 0;
    void setLookahedDistance(double d);

    void visualisationOn();
    void visualisationOff();
    void publishMarkerTargetCurrent(geometry_msgs::PoseStamped t_pose);
    
};

#endif // PUSHPLANNER_H
