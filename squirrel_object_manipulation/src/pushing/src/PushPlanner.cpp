#include "../include/PushPlanner.hpp"

#include<iostream>
#include<armadillo>

using namespace std;
using namespace arma;

int sgn(double d){
    return d<0 ? -1:1;
}

PushPlanner::PushPlanner()
{
    this->push_state_ = INACTIVE;
    this->push_active_ = false;
}

PushPlanner::PushPlanner(string local_frame_, string global_frame_, geometry_msgs::Pose2D pose_robot_, geometry_msgs::PoseStamped pose_object_, nav_msgs::Path pushing_path_, double lookahead_, double goal_toll_, bool state_machine_, double controller_frequency_):
    private_nh("~")
{
    this->initialize(local_frame_, global_frame_, pose_robot_, pose_object_, pushing_path_, lookahead_, goal_toll_, state_machine_, controller_frequency_);


}

void PushPlanner::initialize(string local_frame_, string global_frame_, geometry_msgs::Pose2D pose_robot_, geometry_msgs::PoseStamped pose_object_, nav_msgs::Path pushing_path_, double lookahead_, double goal_toll_, bool state_machine_, double controller_frequency_){

    this->global_frame_ = global_frame_;
    this->local_frame_ = local_frame_;
    this->pose_robot_ = pose_robot_;
    this->pose_object_ = pose_object_;
    this->pushing_path_ = pushing_path_;
    this->goal_reached_ = false;
    this->push_active_ = false;
    this->goal_toll_ = goal_toll_;
    this->state_machine_ = state_machine_;
    this->controller_frequency_ = controller_frequency_;
    this->time_step_ = 1 / controller_frequency_;
    this->push_state_ = INACTIVE;
    this->goal_ = pushing_path_.poses[pushing_path_.poses.size() - 1];

    vis_points_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/push_action/push_markers", 10, true);
    marker_target_c_ = nh.advertise<visualization_msgs::Marker>("/push_action/current_target", 10, true);
    marker_object_c_ = nh.advertise<visualization_msgs::Marker>("/push_action/current_object_pose", 10, true);
    marker_point_ = nh.advertise<visualization_msgs::Marker>("/push_action/point", 10, true);
    visualise_ = false;

    setLookahedDistance(lookahead_);

    this->initChild();
}

void PushPlanner::updatePushPlanner(geometry_msgs::Pose2D pose_robot_, geometry_msgs::PoseStamped pose_object_){

    this->pose_robot_ = pose_robot_;
    this->pose_object_ = pose_object_;

    current_target_ = this->getLookaheadPoint();

    //translation error object-goal
    double dO2G = distancePoints(pose_object_.pose.position.x, pose_object_.pose.position.y, goal_ .pose.position.x, goal_.pose.position.y);

    switch (push_state_){

    case RELOCATE:
    {
        if(!rel_){
            relocate_target_ = reflectPointOverPoint(pose_object_.pose.position.x, pose_object_.pose.position.y,  current_target_.pose.position.x, current_target_.pose.position.y);
            double aO2P = atan2(current_target_.pose.position.y - pose_object_.pose.position.y, current_target_.pose.position.x - pose_object_.pose.position.x);
            if (isnan(aO2P)) aO2P = 0;
            relocate_target_orient_ = aO2P;
            //error_orient_sgn_ = sgn(relocate_target_orient_ - pose_robot_.theta);
            rel_ = true;
        }
        geometry_msgs::PoseStamped pp = pose_object_;
        pp.pose.position.x = relocate_target_ (0);
        pp.pose.position.y = relocate_target_ (1);
        publishPoint(pp);

        double aO2P = atan2(current_target_.pose.position.y - pose_object_.pose.position.y, current_target_.pose.position.x - pose_object_.pose.position.x);
        if (isnan(aO2P)) aO2P = 0;
        cout <<" aO2P "<< aO2P<<endl;
        cout <<"robot orient "<< pose_robot_.theta<<endl;

        if ((distancePoints(pose_robot_.x, pose_robot_.y, relocate_target_ (0), relocate_target_ (1)) < 0.06) && (fabs(relocate_target_orient_ - pose_robot_.theta)<0.1))
            push_state_ = PUSH;

    }
        break;

    case PUSH:
    {

        if (dO2G < goal_toll_){
            goal_reached_ = true;
            push_state_ = INACTIVE;
        }

    }
        break;

    case INACTIVE:
    {
        push_active_ = false;

    }
        break;

    }


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
        p_lookahead = pushing_path_.poses.size() - 1;
    }
    else{

        for (size_t i = p_min_ind; i < pushing_path_.poses.size(); i++) {

            double d_curr = distancePoints(pushing_path_.poses[i].pose.position.x, pushing_path_.poses[i].pose.position.y, pushing_path_.poses[p_min_ind].pose.position.x, pushing_path_.poses[p_min_ind].pose.position.y);
            if (abs(d_curr - lookahead_) < neighbourhood_min){
                neighbourhood_min = abs(d_curr - lookahead_);
                p_lookahead = i;
            }
        }
    }

    if (visualise_){
        publishMarkerTargetCurrent(pushing_path_.poses[p_lookahead]);
        publishMarkerObjectCurrent(pose_object_);

    }

    return pushing_path_.poses[p_lookahead];
}

void PushPlanner::setLookahedDistance(double d){

    lookahead_ = d;
}

void PushPlanner::startPush(){

    if (push_state_ == INACTIVE)
        push_state_ = RELOCATE;

    push_active_ = true;
    goal_reached_ = false;

    rel_ = false;

}

void PushPlanner::stopPush(){

    push_state_ = INACTIVE;
    push_active_  = false;
}

geometry_msgs::Twist PushPlanner::getControlCommand(){

    if (push_state_ == INACTIVE)
        return getNullTwist();

    if (push_state_ == RELOCATE)
        return relocateVelocities();

    if (push_state_ == PUSH)
        return this->getVelocities();

}

geometry_msgs::Twist PushPlanner::relocateVelocities(){
    geometry_msgs::Twist cmd = getNullTwist();

    double attraction_coefficient = 0.7;
    double repulsion_coefficient = 0.7;
    double repulsion_threshold = 0.5;
    double max_linear_velocity = 0.6;
    double max_angular_velocity = 0.6;
    double rotation_coefficient = 0.6;

    // Attraction
    double G_attr_x = -attraction_coefficient*(pose_robot_.x - relocate_target_(0));
    double G_attr_y = -attraction_coefficient*(pose_robot_.y - relocate_target_(1));

    // Repulsion
    double distance = distancePoints(pose_robot_.x, pose_robot_.y, pose_object_.pose.position.x, pose_object_.pose.position.y);
    double G_rep_x, G_rep_y;
    if (distance < repulsion_threshold){
        G_rep_x =  -repulsion_coefficient*(pose_robot_.x - pose_object_.pose.position.x)*(1/pow(distance,2)-repulsion_threshold/pow(distance,3));
        G_rep_y =  -repulsion_coefficient*(pose_robot_.y - pose_object_.pose.position.y)*(1/pow(distance,2)-repulsion_threshold/pow(distance,3));
    }
    else {
        G_rep_x = 0;
        G_rep_y = 0;

    }
    double G_x = G_attr_x + G_rep_x;
    double G_y = G_attr_y + G_rep_y;

    double vel_x;
    double vel_y;

    if(fabs(G_x) > max_linear_velocity) vel_x = (G_x> 0 ? max_linear_velocity : -max_linear_velocity);
    else cmd.linear.x = G_x;
    if(fabs(G_y) > max_linear_velocity) vel_y = (G_y> 0 ? max_linear_velocity : -max_linear_velocity);
    else cmd.linear.y = G_y;


    vec vel_R_  = rotate2DVector(G_x ,G_y, -pose_robot_.theta);
    cmd.linear.x = vel_R_(0);
    cmd.linear.y = vel_R_(1);

    //angle between object pose and target
    double aO2P = atan2(current_target_.pose.position.y - pose_object_.pose.position.y, current_target_.pose.position.x - pose_object_.pose.position.x);
    if (isnan(aO2P)) aO2P = 0;
   // if (aO2P < 0) aO2P = 2 * M_PI + aO2P;

    //orientation cmd
    double theta_r_ = pose_robot_.theta;
    //if (theta_r_ < 0) theta_r_ = 2 * M_PI + theta_r_;
    //if (theta_r_ < -2.5) theta_r_ = 2 * M_PI + theta_r_;
    //double vel_th = -rotation_coefficient * error_orient_sgn_ * fabs(aO2P - theta_r_);
    double vel_th = -rotation_coefficient * fabs(aO2P - theta_r_);


    if(fabs(vel_th) > max_angular_velocity) cmd.angular.z = (vel_th> 0 ? max_angular_velocity : -max_angular_velocity);
    else cmd.angular.z = vel_th;


    cout<<cmd<<endl;

    return cmd;

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


