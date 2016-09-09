#ifndef PUSHPLANNER_H
#define PUSHPLANNER_H

#include <string>
#include <limits>
#include <math.h>
#include <ros/ros.h>
#include <armadillo>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

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
    APPROACH,
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

    bool rel_;

    ros::Publisher vis_corridor_;
    ros::Publisher marker_target_c_;
    ros::Publisher marker_object_c_;
    ros::Publisher marker_robot_c_;
    ros::Publisher marker_point_;

    void publishWaypointMarkerArray(ros::NodeHandle nh);
    void updateMatrix();


protected:

    bool state_machine_;

    PushState push_state_;
    ros::NodeHandle  private_nh;
    ros::Publisher pushing_plan_pub_;
    double object_diameter_, robot_diameter_, corridor_width_;

    bool visualise_;

    double err_t_toll_;
    double err_th_toll_;
    double vel_lin_max_, vel_lin_min_;
    double vel_ang_max_;
    double vel_x_max_, vel_y_max_;
    double controller_frequency_, time_step_;

    double aO2P, aR2O, aR2P, aORP;
    double dO2P, dR2O, dRlOT;
    double zeta;

    string experimentName;


    geometry_msgs::Pose2D pose_robot_;
    geometry_msgs::PoseStamped pose_object_, previous_pose_object_;
    geometry_msgs::PoseStamped current_target_, previous_target_;


    arma::mat pose_robot_vec_;
    arma::mat pose_object_vec_;
    arma::mat current_target_vec_;
    arma::mat relocate_target_vec_;
    arma::vec relocate_target_;
    arma::mat current_time_vec_;
    int elem_count_;

    double current_time_, start_time_;

    geometry_msgs::PoseStamped goal_;
    nav_msgs::Path pushing_path_;

    geometry_msgs::PoseStamped getLookaheadPoint();
    geometry_msgs::PoseStamped getLookaheadPoint(geometry_msgs::PoseStamped pose_object_);

    geometry_msgs::PoseStamped getLookaheadPointDynamic();
    geometry_msgs::PoseStamped getLookaheadPointDynamic(geometry_msgs::PoseStamped pose_object_);

    ros::NodeHandle nh;

    virtual void initChild() = 0;
    virtual void updateChild() = 0;

public:

    bool goal_reached_;
    bool push_active_;

    PushPlanner();
    PushPlanner(string local_frame_, string global_frame_, geometry_msgs::Pose2D pose_robot_, geometry_msgs::PoseStamped pose_object_, nav_msgs::Path pushing_path_, double lookahead_,  double goal_toll_, bool state_machine_, double controller_frequency_, double object_diameter_, double robot_diameter_, double corridor_width_);
    void initialize(string local_frame_, string global_frame_, geometry_msgs::Pose2D pose_robot_, geometry_msgs::PoseStamped pose_object_, nav_msgs::Path pushing_path_, double lookahead_, double goal_toll_, bool state_machine_, double controller_frequency_, double object_diameter_, double robot_diameter_, double corridor_width_);

    virtual void updatePushPlanner(geometry_msgs::Pose2D pose_robot_, geometry_msgs::PoseStamped pose_object_);
    virtual geometry_msgs::Twist getVelocities() = 0;

    geometry_msgs::Twist getControlCommand();
    void setLookahedDistance(double d);

    geometry_msgs::Twist relocateVelocities();
    geometry_msgs::Twist approachVelocities();


    void startPush();
    void stopPush();

    //visualisation

    void plotData();

    void visualisationOn();
    void visualisationOff();
    void publishMarkerTargetCurrent(geometry_msgs::PoseStamped t_pose);
    void publishMarkerObjectCurrent(geometry_msgs::PoseStamped t_pose);
    void publishMarkerRobotCurrent(geometry_msgs::Pose2D t_pose);
    void publishPoint(geometry_msgs::PoseStamped t_pose);
    void publishPoint(arma::vec t);
    void publishCorridor();
    void deleteMarkers();

    void saveData(string path);
    virtual void saveDataChild(string path) = 0;
    void setExperimentName(string name);

};

#endif // PUSHPLANNER_H
