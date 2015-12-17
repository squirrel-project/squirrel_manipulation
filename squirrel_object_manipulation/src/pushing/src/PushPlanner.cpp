#include "../include/PushPlanner.hpp"

PushPlanner::PushPlanner()
{
}

PushPlanner::PushPlanner(string local_frame_, string global_frame_, geometry_msgs::Pose2D pose_robot_, geometry_msgs::PoseStamped pose_object_, nav_msgs::Path pushing_path_, double lookahead_)
{
    this->initialize(local_frame_, global_frame_, pose_robot_, pose_object_, pushing_path_, lookahead_);


}

void PushPlanner::initialize(string local_frame_, string global_frame_, geometry_msgs::Pose2D pose_robot_, geometry_msgs::PoseStamped pose_object_, nav_msgs::Path pushing_path_, double lookahead_){
    this->global_frame_ = global_frame_;
    this->local_frame_ = local_frame_;
    this->pose_robot_ = pose_robot_;
    this->pose_object_ = pose_object_;
    this->pushing_path_ = pushing_path_;
    this->goal_reached_ = false;

    vis_points_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/push_action/push_markers", 10, true);
    marker_target_c_ = nh.advertise<visualization_msgs::Marker>("/push_action/current_target", 10, true);
    visualise_ = false;

    setLookahedDistance(lookahead_);
}

geometry_msgs::PoseStamped PushPlanner::getLookaheadPoint(){

    //getting the closests point on path
    int p_min_ind = 0;
    double d_min = std::numeric_limits<double>::infinity();

    for(size_t i = 0; i < pushing_path_.poses.size(); i++) {

        double d_curr = distancePoints(pushing_path_.poses[i].pose.position.x, pushing_path_.poses[i].pose.position.y, pose_object_.pose.position.x, pose_object_.pose.position.y);
        if (d_min > d_curr){
            p_min_ind = i;
            d_min = d_curr;
        }
    }

    if (visualise_)publishMarkerTargetCurrent(pushing_path_.poses[p_min_ind]);

    //determining the target point with lookahead distance
    double neighbourhood_min = std::numeric_limits<double>::infinity();
    int p_lookahead = p_min_ind;

    if(distancePoints(pushing_path_.poses[pushing_path_.poses.size() - 1].pose.position.x, pushing_path_.poses[pushing_path_.poses.size() - 1].pose.position.y, pushing_path_.poses[p_min_ind].pose.position.x, pushing_path_.poses[p_min_ind].pose.position.y) < lookahead_){
        p_lookahead = pushing_path_.poses.size() - 1;
    }
    else{

        for (size_t i = p_min_ind; i < pushing_path_.poses.size(); i++) {
            double d_curr = distancePoints(pushing_path_.poses[i].pose.position.x, pushing_path_.poses[i].pose.position.y, pushing_path_.poses[p_min_ind].pose.position.x, pushing_path_.poses[p_min_ind].pose.position.y);
            if ((abs(d_curr) - lookahead_) < neighbourhood_min){
                neighbourhood_min = abs(d_curr);
                p_lookahead = i;
            }
        }
    }
    //if (visualise_)publishMarkerTargetCurrent(pushing_path_.poses[p_lookahead]);

    return pushing_path_.poses[p_lookahead];
}

void PushPlanner::setLookahedDistance(double d){

    lookahead_ = d;
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
