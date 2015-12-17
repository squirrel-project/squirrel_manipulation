#include "../include/PushPlanner.hpp"

PushPlanner::PushPlanner()
{
}

PushPlanner::PushPlanner(string local_frame_, string global_frame_, geometry_msgs::Pose2D pose_robot_, geometry_msgs::PoseStamped pose_object_, nav_msgs::Path pushing_path_, double lookahead_)
{
    this->initialize(local_frame_, global_frame_, pose_robot_, pose_object_, pushing_path_, lookahead_);

    // visualisation
    vis_points_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/push_action/push_markers", 10, true);

}

void PushPlanner::initialize(string local_frame_, string global_frame_, geometry_msgs::Pose2D pose_robot_, geometry_msgs::PoseStamped pose_object_, nav_msgs::Path pushing_path_, double lookahead_){
    this->global_frame_ = global_frame_;
    this->local_frame_ = local_frame_;
    this->pose_robot_ = pose_robot_;
    this->pose_object_ = pose_object_;
    this->pushing_path_ = pushing_path_;
    this->goal_reached_ = false;

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

    //determining the target point with lookahead distance
    double neighbourhood_min = std::numeric_limits<double>::infinity();
    int p_lookahead = p_min_ind;

    if(distancePoints(pushing_path_.poses[pushing_path_.poses.size() - 1].pose.position.x, pushing_path_.poses[pushing_path_.poses.size() - 1].pose.position.y, pushing_path_.poses[p_min_ind].pose.position.x, pushing_path_.poses[p_min_ind].pose.position.y) < lookahead_){
        return pushing_path_.poses[pushing_path_.poses.size() - 1];
    }
    else{

        for (size_t i = p_min_ind; i < pushing_path_.poses.size(); i++) {
            double d_curr = distancePoints(pushing_path_.poses[i].pose.position.x, pushing_path_.poses[i].pose.position.y, pushing_path_.poses[p_min_ind].pose.position.x, pushing_path_.poses[p_min_ind].pose.position.y);
            if ((abs(d_curr) - lookahead_) < neighbourhood_min){
                neighbourhood_min = abs(d_curr);
                p_lookahead = i;
            }
        }

        return pushing_path_.poses[p_lookahead];
    }
}

void PushPlanner::setLookahedDistance(double d){

    lookahead_ = d;
}
