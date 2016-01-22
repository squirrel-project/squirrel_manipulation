#include "../include/PushPlanner.hpp"

#include<iostream>
#include<armadillo>

using namespace std;
using namespace arma;

void PushPlanner::plotData(){

}

void PushPlanner::visualisationOn(){

    visualise_ = true;
}

void PushPlanner::visualisationOff(){

    visualise_ = false;
}

void PushPlanner::publishMarkerTargetCurrent(geometry_msgs::PoseStamped t_pose) {

    visualization_msgs::Marker marker;
    marker.header.frame_id = t_pose.header.frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "push_action";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = t_pose.pose;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker_target_c_.publish(marker);
}

void PushPlanner::publishMarkerObjectCurrent(geometry_msgs::PoseStamped t_pose) {

    visualization_msgs::Marker marker;
    marker.header.frame_id = t_pose.header.frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "push_action";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = t_pose.pose;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker_object_c_.publish(marker);
}

void PushPlanner::publishPoint(geometry_msgs::PoseStamped t_pose) {

    visualization_msgs::Marker marker;
    marker.header.frame_id = t_pose.header.frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "push_action";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = t_pose.pose;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker_point_.publish(marker);
}

void PushPlanner::publishPoint(vec t){
    geometry_msgs::PoseStamped pp = pose_object_;
    pp.pose.position.x = t(0);
    pp.pose.position.y = t(1);
    publishPoint(pp);
}

