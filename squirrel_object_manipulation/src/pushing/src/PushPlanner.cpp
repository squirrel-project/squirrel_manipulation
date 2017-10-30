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

PushPlanner::PushPlanner(string local_frame_, string global_frame_, geometry_msgs::Pose2D pose_robot_, geometry_msgs::PoseStamped pose_object_, nav_msgs::Path pushing_path_, double lookahead_, double goal_toll_, bool state_machine_, double controller_frequency_, double object_diameter_, double robot_diameter_, double corridor_width_, vector<double> corridor_width_array_,  bool , bool relaxation_):
    private_nh("~")
{
    this->initialize(local_frame_, global_frame_, pose_robot_, pose_object_, pushing_path_, lookahead_, goal_toll_, state_machine_, controller_frequency_, object_diameter_, robot_diameter_, corridor_width_, corridor_width_array_, fixed_, relaxation_);
    
}

void PushPlanner::initialize(string local_frame_, string global_frame_, geometry_msgs::Pose2D pose_robot_, geometry_msgs::PoseStamped pose_object_, nav_msgs::Path pushing_path_, double lookahead_, double goal_toll_, bool state_machine_, double controller_frequency_, double object_diameter_, double robot_diameter_, double corridor_width_, vector<double> corridor_width_array_,  bool fixed_, bool relaxation_){
    
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
    this->object_diameter_= object_diameter_;
    this->robot_diameter_ = robot_diameter_;
    this->corridor_width_ = corridor_width_;
    this->time_step_ = 1 / controller_frequency_;
    this->push_state_ = INACTIVE;
    this->goal_ = pushing_path_.poses[pushing_path_.poses.size() - 1];
    this->corridor_width_array_ = corridor_width_array_;
    this->fixed_ = fixed_;
    this->relaxation_ = relaxation_;
    this->sim_ = false;




    edge_push_corridor_p_.header = pushing_path_.header;
    edge_push_corridor_n_.header = pushing_path_.header;
    edge_object_corridor_p_.header = pushing_path_.header;
    edge_object_corridor_n_.header = pushing_path_.header;

    //calculating push corridor edges
    double smooth = 0.75;
    for (int i = 0; i < this->corridor_width_array_.size() - 1; i++){
        geometry_msgs::PoseStamped p = pushing_path_.poses[i];
        geometry_msgs::PoseStamped p1 = pushing_path_.poses[i];
        geometry_msgs::PoseStamped p2 = pushing_path_.poses[i + 1];

        p.pose.position.x =  parallelCurveWidthTrans(p1.pose.position.x, p1.pose.position.x - p2.pose.position.x, p1.pose.position.y, p1.pose.position.y - p2.pose.position.y, true, corridor_width_array_.at(i) / 2);
        p.pose.position.y =  parallelCurveWidthTrans(p1.pose.position.y, p1.pose.position.y - p2.pose.position.y, p1.pose.position.x, p1.pose.position.x - p2.pose.position.x, false, corridor_width_array_.at(i) / 2);

        sleep(0.5);
        if (i > 0){
            p.pose.position.x = smooth * edge_push_corridor_p_.poses.at(i - 1).pose.position.x  + (1 - smooth) * parallelCurveWidthTrans(p1.pose.position.x, p1.pose.position.x - p2.pose.position.x, p1.pose.position.y, p1.pose.position.y - p2.pose.position.y, true, corridor_width_array_.at(i) / 2);
            p.pose.position.y = smooth * edge_push_corridor_p_.poses.at(i - 1).pose.position.y  + (1 - smooth) * parallelCurveWidthTrans(p1.pose.position.y, p1.pose.position.y - p2.pose.position.y,p1.pose.position.x, p1.pose.position.x - p2.pose.position.x, false, corridor_width_array_.at(i) / 2);

            if ((distancePoints(p.pose.position.x, p.pose.position.y, edge_push_corridor_p_.poses.at(i - 1).pose.position.x, edge_push_corridor_p_.poses.at(i - 1).pose.position.y) > 0.1)) {
                p =  edge_push_corridor_p_.poses.at(i - 1);
            }
        }
        edge_push_corridor_p_.poses.push_back(p);
        if (i == this->corridor_width_array_.size() - 2) edge_push_corridor_p_.poses.push_back(p);

        p.pose.position.x = parallelCurveWidthTrans(p1.pose.position.x, p1.pose.position.x - p2.pose.position.x, p1.pose.position.y, p1.pose.position.y - p2.pose.position.y, false, corridor_width_array_.at(i) / 2);
        p.pose.position.y = parallelCurveWidthTrans(p1.pose.position.y, p1.pose.position.y - p2.pose.position.y, p1.pose.position.x, p1.pose.position.x - p2.pose.position.x, true, corridor_width_array_.at(i) / 2);
        if (i > 0){
            p.pose.position.x = smooth * edge_push_corridor_n_.poses.at(i - 1).pose.position.x  + (1 - smooth) * parallelCurveWidthTrans(p1.pose.position.x, p1.pose.position.x - p2.pose.position.x, p1.pose.position.y, p1.pose.position.y - p2.pose.position.y, false, corridor_width_array_.at(i) / 2);
            p.pose.position.y = smooth * edge_push_corridor_n_.poses.at(i - 1).pose.position.y  + (1 - smooth) * parallelCurveWidthTrans(p1.pose.position.y, p1.pose.position.y - p2.pose.position.y,p1.pose.position.x, p1.pose.position.x - p2.pose.position.x, true, corridor_width_array_.at(i) / 2);

            if ((distancePoints(p.pose.position.x, p.pose.position.y, edge_push_corridor_n_.poses.at(i - 1).pose.position.x, edge_push_corridor_n_.poses.at(i - 1).pose.position.y) > 0.1)) {
                p =  edge_push_corridor_n_.poses.at(i - 1);
            }
        }
        edge_push_corridor_n_.poses.push_back(p);
        if (i == this->corridor_width_array_.size() - 2) edge_push_corridor_n_.poses.push_back(p);
    }


    //calculating object corridor edges
    for (int i = 0; i < this->corridor_width_array_.size() - 2; i++){
        geometry_msgs::PoseStamped p = edge_push_corridor_p_.poses[i];
        geometry_msgs::PoseStamped p1 = edge_push_corridor_p_.poses[i];
        geometry_msgs::PoseStamped p2 = edge_push_corridor_p_.poses[i + 1];
        double x1  = parallelCurveWidthTrans(p1.pose.position.x, p1.pose.position.x - p2.pose.position.x, p1.pose.position.y, p1.pose.position.y - p2.pose.position.y, false, robot_diameter_ + object_diameter_ / 2);
        double y1 = parallelCurveWidthTrans(p1.pose.position.y, p1.pose.position.y - p2.pose.position.y,p1.pose.position.x, p1.pose.position.x - p2.pose.position.x, true, robot_diameter_ + object_diameter_ / 2);
        double x2  = parallelCurveWidthTrans(p1.pose.position.x, p1.pose.position.x - p2.pose.position.x, p1.pose.position.y, p1.pose.position.y - p2.pose.position.y, true, robot_diameter_ + object_diameter_ / 2);
        double y2 = parallelCurveWidthTrans(p1.pose.position.y, p1.pose.position.y - p2.pose.position.y,p1.pose.position.x, p1.pose.position.x - p2.pose.position.x, false, robot_diameter_ + object_diameter_ / 2);
        double x,y;
        if (distancePoints(x1, y1, pushing_path_.poses.at(i).pose.position.x, pushing_path_.poses.at(i).pose.position.y) < distancePoints(x2, y2, pushing_path_.poses.at(i).pose.position.x, pushing_path_.poses.at(i).pose.position.y)){
            x = x1;
            y = y1;
        }
        else {
            x = x2;
            y = y2;
        }
        if (i > 0){
            p.pose.position.x = smooth * edge_object_corridor_p_.poses.at(i - 1).pose.position.x + (1 - smooth) * x;
            p.pose.position.y = smooth * edge_object_corridor_p_.poses.at(i - 1).pose.position.y + (1 - smooth) * y;

            if ((distancePoints(p.pose.position.x, p.pose.position.y, edge_object_corridor_p_.poses.at(i - 1).pose.position.x, edge_object_corridor_p_.poses.at(i - 1).pose.position.y) > 0.1)) {
                p =  edge_object_corridor_p_.poses.at(i - 1);
            }
        }
        else {
            p.pose.position.x = x;
            p.pose.position.y = y;
        }
        edge_object_corridor_p_.poses.push_back(p);
        if (i == this->corridor_width_array_.size() - 3){
            edge_object_corridor_p_.poses.push_back(p);
            edge_object_corridor_p_.poses.push_back(p);
        }

        p = edge_push_corridor_n_.poses[i];
        p1 = edge_push_corridor_n_.poses[i];
        p2 = edge_push_corridor_n_.poses[i + 1];
        x1 = parallelCurveWidthTrans(p1.pose.position.x, p1.pose.position.x - p2.pose.position.x, p1.pose.position.y, p1.pose.position.y - p2.pose.position.y, true, robot_diameter_ + object_diameter_ / 2);
        y1 = parallelCurveWidthTrans(p1.pose.position.y, p1.pose.position.y - p2.pose.position.y,p1.pose.position.x, p1.pose.position.x - p2.pose.position.x, false, robot_diameter_ + object_diameter_ / 2);
        x2 = parallelCurveWidthTrans(p1.pose.position.x, p1.pose.position.x - p2.pose.position.x, p1.pose.position.y, p1.pose.position.y - p2.pose.position.y, false, robot_diameter_ + object_diameter_ / 2);
        y2 = parallelCurveWidthTrans(p1.pose.position.y, p1.pose.position.y - p2.pose.position.y,p1.pose.position.x, p1.pose.position.x - p2.pose.position.x, true, robot_diameter_ + object_diameter_ / 2);
        if (distancePoints(x1, y1, pushing_path_.poses.at(i).pose.position.x, pushing_path_.poses.at(i).pose.position.y) < distancePoints(x2, y2, pushing_path_.poses.at(i).pose.position.x, pushing_path_.poses.at(i).pose.position.y)){
            x = x1;
            y = y1;
        }
        else {
            x = x2;
            y = y2;
        }
        if (i > 0){
            p.pose.position.x = smooth * edge_object_corridor_n_.poses.at(i - 1).pose.position.x + (1 - smooth) * x;
            p.pose.position.y = smooth * edge_object_corridor_n_.poses.at(i - 1).pose.position.y + (1 - smooth) * y;

            if ((distancePoints(p.pose.position.x, p.pose.position.y, edge_object_corridor_n_.poses.at(i - 1).pose.position.x, edge_object_corridor_n_.poses.at(i - 1).pose.position.y) > 0.1)) {
                p =  edge_object_corridor_n_.poses.at(i - 1);
            }
        }
        else {
            p.pose.position.x = x;
            p.pose.position.y = y;
        }
        edge_object_corridor_n_.poses.push_back(p);
        if (i == this->corridor_width_array_.size() - 3) {
            edge_object_corridor_n_.poses.push_back(p);
            edge_object_corridor_n_.poses.push_back(p);
        }
    }



    corridor_object_width_array_.clear();
    for (int i = 0; i < this->corridor_width_array_.size(); i++){
        corridor_object_width_array_.push_back(corridor_width_array_.at(i) - 2 * robot_diameter_ - object_diameter_);
    }

    vis_corridor_ = nh.advertise<visualization_msgs::MarkerArray>("/push_action/push_corridor", 100, true);
    vis_object_corridor_ = nh.advertise<visualization_msgs::MarkerArray>("/push_action/object_corridor", 100, true);
    marker_target_c_ = nh.advertise<visualization_msgs::Marker>("/push_action/current_target", 100, true);
    marker_object_c_ = nh.advertise<visualization_msgs::Marker>("/push_action/current_object_pose", 100, true);
    marker_robot_c_ = nh.advertise<visualization_msgs::Marker>("/push_action/current_robot_pose", 100, true);
    marker_point_ = nh.advertise<visualization_msgs::Marker>("/push_action/point", 100, true);
    pushing_plan_pub_ = nh.advertise<nav_msgs::Path>("/push_action/pushing_path", 1000, true);
    pushing_edge_p_pub_ = nh.advertise<nav_msgs::Path>("/push_action/pushing_edge_p", 1000, true);
    pushing_edge_n_pub_ = nh.advertise<nav_msgs::Path>("/push_action/pushing_edge_n", 1000, true);
    object_edge_p_pub_ = nh.advertise<nav_msgs::Path>("/push_action/object_edge_p", 1000, true);
    object_edge_n_pub_ = nh.advertise<nav_msgs::Path>("/push_action/object_edge_n", 1000, true);



    visualise_ = true;

    pose_robot_vec_.set_size(pushing_path_.poses.size(), 3);
    current_time_vec_.set_size(pushing_path_.poses.size(), 1);
    pose_object_vec_.set_size(pushing_path_.poses.size(), 3);
    current_target_vec_.set_size(pushing_path_.poses.size(), 3);
    elem_count_ = 0;

    setLookahedDistance(lookahead_);

    start_time_ = ros::Time::now().toSec();

    zeta = corridor_width_ /  (2 * (robot_diameter_ + object_diameter_));

    if(zeta > 1) zeta = 1;

    this->initChild();
}

void PushPlanner::updatePushPlanner(geometry_msgs::Pose2D pose_robot_, geometry_msgs::PoseStamped pose_object_){

    this->pose_robot_ = pose_robot_;
    this->pose_object_ = pose_object_;
    this->previous_target_ = this->current_target_;

    //if(!fixed_)this->current_target_ = this->getLookaheadPointDynamicFlex();
    //else if(this->pushpoint_)this->current_target_ = pushing_path_.poses.at(pushing_path_.poses.size());
    //else
    this->current_target_ = this->getLookaheadPointFixedDistance();
    //this->current_target_ = this->getLookaheadPointDynamic();

    //this->current_target_ =  pushing_path_.poses[pushing_path_.poses.size()-1];
    this->current_time_ = ros::Time::now().toSec();

    if (visualise_){




        publishMarkerTargetCurrent(current_target_);
        publishMarkerObjectCurrent(pose_object_);
        publishMarkerRobotCurrent(pose_robot_);
        publishCorridor();
        pushing_plan_pub_.publish(pushing_path_);
        pushing_edge_p_pub_.publish(edge_push_corridor_p_);
        pushing_edge_n_pub_.publish(edge_push_corridor_n_);
        object_edge_p_pub_.publish(edge_object_corridor_p_);
        object_edge_n_pub_.publish(edge_object_corridor_n_);

    }


    this->updateMatrix();

    //the angle of a vector object-target point
    aO2P = getVectorAngle(current_target_.pose.position.x - pose_object_.pose.position.x, current_target_.pose.position.y - pose_object_.pose.position.y);

    //the angle of a vector robot-object
    aR2O = getVectorAngle(pose_object_.pose.position.x - pose_robot_.x, pose_object_.pose.position.y - pose_robot_.y);

    //the angle of a vector robot-target
    aR2P = getVectorAngle(current_target_.pose.position.x - pose_robot_.x, current_target_.pose.position.y - pose_robot_.y);

    //the angle object-robot-target
    aORP =  angle3Points(pose_object_.pose.position.x, pose_object_.pose.position.y, pose_robot_.x, pose_robot_.y, current_target_.pose.position.x, current_target_.pose.position.y);

    //translation error object-goal
    double dO2G = distancePoints(pose_object_.pose.position.x, pose_object_.pose.position.y, goal_ .pose.position.x, goal_.pose.position.y);

    //distance object target point
    dO2P = distancePoints(pose_object_.pose.position.x, pose_object_.pose.position.y, current_target_.pose.position.x, current_target_.pose.position.y);

    //distance robot object
    dR2O = distancePoints(pose_robot_.x, pose_robot_.y, pose_object_.pose.position.x, pose_object_.pose.position.y);

    //distance robot to the line object-target
    dRlOT = distance2Line(pose_robot_.x, pose_robot_.y, pose_object_.pose.position.x, pose_object_.pose.position.y, current_target_.pose.position.x, current_target_.pose.position.y);


    switch (push_state_){

    case RELOCATE:
    {
        if(!rel_){
            //relocate_target_vec_.set_size(3,1);
            relocate_target_.set_size(3);
            relocate_target_(span(0,1)) = pointOnLineWithDistanceFromPointOuter(pose_object_.pose.position.x, pose_object_.pose.position.y,  current_target_.pose.position.x, current_target_.pose.position.y, object_diameter_ / 2 + robot_diameter_ / 2);
            relocate_target_(2) = aO2P;
            rel_ = true;

        }

        //relocate_target_vec_(span(0,1),relocate_target_vec_.n_cols - 1) = pointOnLineWithDistanceFromPointOuter(pose_object_.pose.position.x, pose_object_.pose.position.y,  current_target_.pose.position.x, current_target_.pose.position.y, object_diameter_ / 2);

        //        relocate_target_vec_(2, relocate_target_vec_.n_cols - 1) = aO2P;
        //        for (int i = 0; i < 3; i ++) relocate_target_(i) = mean(relocate_target_vec_.row(i));
        //        relocate_target_vec_.resize(3,relocate_target_vec_.n_cols + 1);

        //if (visualise_)publishPoint(relocate_target_);

        if ((distancePoints(pose_robot_.x, pose_robot_.y, relocate_target_ (0), relocate_target_ (1)) < 0.1) && (rotationDifference(aR2O, pose_robot_.theta) < 0.05) ){
            push_state_ = PUSH;
            ROS_INFO("(Push) State: PUSH");
            cout << endl;
        }

        //        if((fabs(aO2P - aR2O) < 0.3) && (fabs(rotationDifference(aR2O, pose_robot_.theta)) < 0.3)){
        //            push_state_ = APPROACH;
        //            ROS_INFO("(Push) State: APPROACH");
        //            cout << endl;
        //        }

    }
        break;

    case APPROACH:
    {
        if (dR2O < 1.2 * object_diameter_){
            push_state_ = PUSH;
            ROS_INFO("(Push) State: PUSH");
            cout << endl;
        }

    }
        break;

    case PUSH:
    {
        this->updateChild();

        if (dO2G < goal_toll_){
            cout<<dO2G<<" and toll "<<goal_toll_<<endl;
            goal_reached_ = true;
            push_state_ = INACTIVE;
        }

        if (((dR2O < robot_diameter_ / 2) || (dR2O > 1.5 * object_diameter_)) && state_machine_){
            cout<<"dR2O "<< dR2O<<endl;
            push_state_ = RELOCATE;
            cout<<"push state: RELOCATE"<<endl;
        }



    }
        break;

    case INACTIVE:
    {
        push_active_ = false;

    }
        break;

    }


    if(push_state_ != RELOCATE)rel_ = false;

}

geometry_msgs::PoseStamped PushPlanner::getLookaheadPoint(geometry_msgs::PoseStamped pose_object_){

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

    return pushing_path_.poses[p_lookahead];
}
geometry_msgs::PoseStamped PushPlanner::getLookaheadPointDynamic(geometry_msgs::PoseStamped pose_object_){

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

    //determining the target point with minimum cost function
    double cost_min = std::numeric_limits<double>::infinity();
    int p_lookahead = p_min_ind;

    if(distancePoints(pushing_path_.poses[pushing_path_.poses.size() - 1].pose.position.x, pushing_path_.poses[pushing_path_.poses.size() - 1].pose.position.y, pushing_path_.poses[p_min_ind].pose.position.x, pushing_path_.poses[p_min_ind].pose.position.y) < lookahead_){
        p_lookahead = pushing_path_.poses.size() - 1;
    }
    else{
        for (size_t i = p_min_ind + 1; i < pushing_path_.poses.size(); i++) {
            double d = distancePoints(pushing_path_.poses[i].pose.position.x, pushing_path_.poses[i].pose.position.y, pose_object_.pose.position.x, pose_object_.pose.position.y);

            double d_max = 0;
            for (size_t j = p_min_ind; j < i; j++) {
                double d_curr = distance2Line(pushing_path_.poses[j].pose.position.x, pushing_path_.poses[j].pose.position.y, pose_object_.pose.position.x, pose_object_.pose.position.y, pushing_path_.poses[i].pose.position.x, pushing_path_.poses[i].pose.position.y);
                if(d_max < d_curr) d_max = d_curr;
            }


            vec ideal_start = pointOnLineWithDistanceFromPointOuter(pose_object_.pose.position.x, pose_object_.pose.position.y,  current_target_.pose.position.x, current_target_.pose.position.y, object_diameter_ / 2 + robot_diameter_ / 2);
            double d_min = std::numeric_limits<double>::infinity();
            for (size_t l = 1; l < i + 1; l++) {
                double d = distancePoints(pushing_path_.poses[l].pose.position.x, pushing_path_.poses[l].pose.position.y, ideal_start(0), ideal_start(1));
                if (d < d_min) d_min = d;
            }
            if (visualise_)publishPoint(ideal_start);

            double penalty_tail =  corridor_width_ / 2 - zeta * (d_min + robot_diameter_ / 2);
            double penalty_curve = corridor_width_ / 2 - zeta * (d_max + robot_diameter_ / 2);

            double cost_curr = 1 / d + zeta * d_max;
            if ((penalty_curve <= 0)||((penalty_tail <= 0))) cost_curr = std::numeric_limits<double>::infinity();
            if (cost_curr < cost_min){
                cost_min = cost_curr;
                p_lookahead = i;
                
            }

        }
    }


    if ((p_lookahead < 3) && (p_min_ind < 3)) return getLookaheadPoint(pose_object_);

    if (p_lookahead == p_min_ind + 1) return current_target_;

    return pushing_path_.poses[p_lookahead];
}

geometry_msgs::PoseStamped PushPlanner::getLookaheadPointDynamicFlex(geometry_msgs::PoseStamped pose_object_){




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


    //getting the closests point on push corridor edges to the object
    double dOEmin = std::numeric_limits<double>::infinity(); //distance to the push corridor edges
    string edge_side ; //edge to which the object is closer n, p
    string edge_side_curr; //edge to which the object is closer n, p
    int edge_min_ind = -1; //closest point on the edge to the object
    double d_curr; //current distance

    for(size_t i = 0; i < edge_push_corridor_n_.poses.size(); i++) {
        if(distancePoints(edge_push_corridor_n_.poses[i].pose.position.x, edge_push_corridor_n_.poses[i].pose.position.y, pose_object_.pose.position.x, pose_object_.pose.position.y) < distancePoints(edge_push_corridor_p_.poses[i].pose.position.x, edge_push_corridor_p_.poses[i].pose.position.y, pose_object_.pose.position.x, pose_object_.pose.position.y)){
            d_curr = distancePoints(edge_push_corridor_n_.poses[i].pose.position.x, edge_push_corridor_n_.poses[i].pose.position.y, pose_object_.pose.position.x, pose_object_.pose.position.y);
            edge_side_curr = "n";
        }
        else{
            d_curr = distancePoints(edge_push_corridor_p_.poses[i].pose.position.x, edge_push_corridor_p_.poses[i].pose.position.y, pose_object_.pose.position.x, pose_object_.pose.position.y);
            edge_side_curr = "p";
        }
        if (dOEmin > d_curr){
            edge_min_ind = i;
            dOEmin = d_curr;
            edge_side = edge_side_curr;
        }

    }


    //storing the point and visualising it
    double edge_point_x, edge_point_y;
    if (edge_side == "n"){
        edge_point_x = edge_push_corridor_n_.poses[edge_min_ind].pose.position.x;
        edge_point_y = edge_push_corridor_n_.poses[edge_min_ind].pose.position.y;
        if (visualise_)publishPoint(edge_push_corridor_n_.poses[edge_min_ind], 4);
        if (visualise_)publishPoint(edge_object_corridor_n_.poses[edge_min_ind], 5);
    }
    else if (edge_side == "p"){
        edge_point_x = edge_push_corridor_p_.poses[edge_min_ind].pose.position.x;
        edge_point_y = edge_push_corridor_p_.poses[edge_min_ind].pose.position.y;
        if (visualise_)publishPoint(edge_push_corridor_p_.poses[edge_min_ind], 4);
        if (visualise_)publishPoint(edge_object_corridor_p_.poses[edge_min_ind], 5);
    }
    else{
        ROS_ERROR("(push - push planner) wrong indices in paths");
        cout<<endl;
        throw;
    }



    //geting tangent line in the point closest to the object on the object edge line
    vec tangent_line(2);
    int p = 0;


    while ((getNorm(tangent_line) == 0)&&(p != 10)){

        if (edge_side == "n"){

            tangent_line(0) = edge_object_corridor_n_.poses.at(edge_min_ind + p).pose.position.x - edge_object_corridor_n_.poses.at(edge_min_ind).pose.position.x;
            tangent_line(1) = edge_object_corridor_n_.poses.at(edge_min_ind + p).pose.position.y - edge_object_corridor_n_.poses.at(edge_min_ind).pose.position.y;

        }
        else{

            tangent_line(0) = edge_object_corridor_p_.poses.at(edge_min_ind + p).pose.position.x - edge_object_corridor_p_.poses.at(edge_min_ind).pose.position.x;
            tangent_line(1) = edge_object_corridor_p_.poses.at(edge_min_ind + p).pose.position.y - edge_object_corridor_p_.poses.at(edge_min_ind).pose.position.y;

        }
        p++;
    }



    //geting point on the path closest to the line formed by object position and closest point on the edge
    d_curr = std::numeric_limits<double>::infinity();
    int path_object_ind = -1;
    for(size_t i = 0; i < pushing_path_.poses.size(); i++) {

        if(distance2Line(pushing_path_.poses[i].pose.position.x, pushing_path_.poses[i].pose.position.y, pose_object_.pose.position.x, pose_object_.pose.position.y,edge_point_x, edge_point_y) < d_curr){

            d_curr = distance2Line(pushing_path_.poses[i].pose.position.x, pushing_path_.poses[i].pose.position.y, pose_object_.pose.position.x, pose_object_.pose.position.y,edge_point_x, edge_point_y);

            path_object_ind = i;
        }
    }

    //calculate ratio  distance object to path / width  in the closest point for angle condition
    double d_object_path = distancePoints(pushing_path_.poses[path_object_ind].pose.position.x, pushing_path_.poses[path_object_ind].pose.position.y, pose_object_.pose.position.x, pose_object_.pose.position.y);
    double ratio_object = d_object_path / (corridor_object_width_array_.at(path_object_ind) / 2);
    if (ratio_object > 1.0) ratio_object = 1.0;


    //determining the target point with minimum cost function
    double cost_max = 0;
    int p_lookahead = 0;

    for (size_t i = 1; i < pushing_path_.poses.size() - 1; i++) {

        // I condition: angle and ratio for penalty object corridor
        vec push_line (2);
        push_line(0) = pushing_path_.poses.at(i).pose.position.x - pose_object_.pose.position.x;
        push_line(1) = pushing_path_.poses.at(i).pose.position.y - pose_object_.pose.position.y;

        double angle = getAngle(push_line,tangent_line);
        if (isnan(angle)) angle = M_PI;

        double penalty_object_corridor = abs(sin(angle)) - ratio_object + 0.1; //0.05 is a tollerance

        // condition  for the relaxed case
        double penalty_tail = 1.0;

        if (relaxation_){
            penalty_tail = -1.0;
            penalty_object_corridor = 1.0;
            vec ideal_start = pointOnLineWithDistanceFromPointOuter(pose_object_.pose.position.x, pose_object_.pose.position.y,  current_target_.pose.position.x, current_target_.pose.position.y, object_diameter_ / 2 + robot_diameter_ / 2);
            if (visualise_ )publishPoint(ideal_start);
             double d_min = std::numeric_limits<double>::infinity();
            if((i>p_min_ind)||(p_min_ind < 10)){
                for (size_t l = p_min_ind; l < i + 1; l++) {

                    double dn = distancePoints(edge_push_corridor_n_.poses[l].pose.position.x, edge_push_corridor_n_.poses[l].pose.position.y, ideal_start(0), ideal_start(1));
                    double dp = distancePoints(edge_push_corridor_p_.poses[l].pose.position.x, edge_push_corridor_p_.poses[l].pose.position.y, ideal_start(0), ideal_start(1));

                    if ((dn < d_min)&&(distancePoints(pushing_path_.poses[l].pose.position.x, pushing_path_.poses[l].pose.position.y, ideal_start(0), ideal_start(1)) < corridor_width_array_.at(l))) d_min = dn;
                    if ((dp < d_min)&&(distancePoints(pushing_path_.poses[l].pose.position.x, pushing_path_.poses[l].pose.position.y, ideal_start(0), ideal_start(1)) < corridor_width_array_.at(l))) d_min = dp;
                }
                penalty_tail = d_min - robot_diameter_;
            }

        }


        // II condition: minimal distance of the push-line from push corridor edges
        double beta = std::numeric_limits<double>::infinity();
        int j_beta = std::numeric_limits<int>::infinity();

        if (i > p_min_ind){
            for (size_t j = p_min_ind + 1; j < i; j++) {
                double d_curr = corridor_width_array_.at(j) / 2 - distance2Segment(pushing_path_.poses[j].pose.position.x, pushing_path_.poses[j].pose.position.y, pose_object_.pose.position.x, pose_object_.pose.position.y, pushing_path_.poses[i].pose.position.x, pushing_path_.poses[i].pose.position.y);
                if(beta > d_curr) {
                    beta = d_curr;
                    j_beta = j;
                }
            }
        }
        else{
            for (size_t j = 0; j < i; j++) {
                double d_curr = corridor_width_array_.at(j) / 2 - distance2Segment(pushing_path_.poses[j].pose.position.x, pushing_path_.poses[j].pose.position.y, pose_object_.pose.position.x, pose_object_.pose.position.y, pushing_path_.poses[i].pose.position.x, pushing_path_.poses[i].pose.position.y);
                if(beta > d_curr) {
                    beta = d_curr;
                    j_beta = j;
                }
            }
        }


        double penalty_curve;
        if (!relaxation_) penalty_curve = beta - (object_diameter_ / 2 + robot_diameter_ /2);
        else penalty_curve = beta - robot_diameter_;
//                publishPoint(pushing_path_.poses.at(i), 1);
//                cout<< " i: "<<i<<" curve  "<<penalty_curve<< " corr "<<penalty_object_corridor<<" tail "<<penalty_tail<<endl;
//                sleep (1);

        // Condition target change

        double penalty_change = 1.0;
       if (abs((getVectorAngle(pushing_path_.poses[current_target_ind_].pose.position.x, pushing_path_.poses[current_target_ind_].pose.position.y) - getVectorAngle(pushing_path_.poses[i].pose.position.x, pushing_path_.poses[i].pose.position.y)))< 1/(object_diameter_ / 2 +robot_diameter_ )) penalty_change = 1.0;

        double cost_curr = i;
        if ((penalty_curve <= 0)||((penalty_object_corridor <= 0))||((penalty_tail <= 0))||((penalty_change <= 0))) cost_curr = 0;
        if (cost_curr > cost_max){
            cost_max = cost_curr;
            p_lookahead = i;

        }

    }


    if (p_lookahead > 0) current_target_ind_ = p_lookahead;

    if (abs(p_lookahead-p_min_ind) < 10) return getLookaheadPointFixedDistance();

    //if(distancePoints(goal_.pose.position.x, goal_.pose.position.y, pose_object_.pose.position.x, pose_object_.pose.position.y) < 0.50) return goal_;

    return pushing_path_.poses[current_target_ind_];
}


geometry_msgs::PoseStamped PushPlanner::getLookaheadPointDynamicFlexApprox(geometry_msgs::PoseStamped pose_object_){


    //getting the closests point on push corridor edges to the object
    double dO2P = std::numeric_limits<double>::infinity(); //distance to the path
    int nO2P = 0; //closest point on the edge to the object
    double d_curr;

    for(int i = 0; i < pushing_path_.poses.size(); i++) {
        d_curr = distancePoints(pushing_path_.poses[i].pose.position.x, pushing_path_.poses[i].pose.position.y, pose_object_.pose.position.x, pose_object_.pose.position.y);
        if (d_curr < dO2P){
            dO2P = d_curr;
            nO2P = i;
        }
    }

    if (visualise_)publishPoint(pushing_path_.poses.at(nO2P), 1);



    //determining the target point with minimum cost function
    double cost_max = 0;
    int p_lookahead = 0;

    //if at the end choose goal
    //if(distancePoints(pushing_path_.poses[pushing_path_.poses.size() - 1].pose.position.x, pushing_path_.poses[pushing_path_.poses.size() - 1].pose.position.y, pushing_path_.poses[nO2P].pose.position.x, pushing_path_.poses[nO2P].pose.position.y) < lookahead_){
    //    p_lookahead = pushing_path_.poses.size() - 1;
    //}
    //cost calculations
    //else{
    for (size_t i = nO2P; i < pushing_path_.poses.size() - 1; i++) {

        //angle condition
        //calculating angle
        vec push_line (2);
        push_line(0) = pushing_path_.poses.at(i).pose.position.x - pose_object_.pose.position.x;
        push_line(1) = pushing_path_.poses.at(i).pose.position.y - pose_object_.pose.position.y;

        //double angle = getAngle(push_line,tangent_line);
        //if (isnan(angle)) angle = M_PI;

        //minimal distance of the push-line from push corridor edges
        //penalty push-line
        double beta = std::numeric_limits<double>::infinity();
        int j_beta = std::numeric_limits<int>::infinity();

        for (size_t j = nO2P; j < i; j++) {
            double d_curr = corridor_width_array_.at(j) / 2 - distance2Line(pushing_path_.poses[j].pose.position.x, pushing_path_.poses[j].pose.position.y, pose_object_.pose.position.x, pose_object_.pose.position.y, pushing_path_.poses[i].pose.position.x, pushing_path_.poses[i].pose.position.y);
            if(beta > d_curr) {
                zeta = corridor_width_array_.at(j) / (2 * robot_diameter_ +  2 * object_diameter_);
                beta = d_curr;
                j_beta = j;
            }
        }
        if(zeta > 1.0) zeta = 1.0;

        double penalty_push_line = beta - zeta * (robot_diameter_);

        //penalty push tail
        //leaving enough space for robot
        vec ideal_start = pointOnLineWithDistanceFromPointOuter(pose_object_.pose.position.x, pose_object_.pose.position.y,  current_target_.pose.position.x, current_target_.pose.position.y, object_diameter_ / 2 + robot_diameter_ / 2);
        if (visualise_)publishPoint(ideal_start);

        double penalty_tail = 1;

        double penalty_object_corridor = 1;

        double cost_curr = i;
        //if ((penalty_curve <= 0)||((penalty_object_corridor <= 0))||((penalty_tail <= 0))) cost_curr = 0;
        if ((penalty_push_line <= 0)||((penalty_object_corridor <= 0))||((penalty_tail <= 0))) cost_curr = 0;
        if (cost_curr > cost_max){
            cost_max = cost_curr;
            p_lookahead = i;
        }

        // }
    }

    if (p_lookahead > 0) current_target_ind_ = p_lookahead;
    //if ((path_object_ind < 10) && (current_target_ind_ < 10)) current_target_ind_ = 10;

    return pushing_path_.poses[current_target_ind_];
    // return pushing_path_.poses[pushing_path_.poses.size() - 1 ];
}
geometry_msgs::PoseStamped PushPlanner::getLookaheadPoint(){
    return this->getLookaheadPoint(this->pose_object_);
}

geometry_msgs::PoseStamped PushPlanner::getLookaheadPointDynamic(){
    return this->getLookaheadPointDynamic(this->pose_object_);
}
geometry_msgs::PoseStamped PushPlanner::getLookaheadPointDynamicFlex(){
    return this->getLookaheadPointDynamicFlex(this->pose_object_);
}

geometry_msgs::PoseStamped PushPlanner::getLookaheadPointFixedDistance(){
    return this->getLookaheadPointFixedDistance(this->pose_object_);
}

void PushPlanner::setLookahedDistance(double d){

    lookahead_ = d;
}

void PushPlanner::startPush(){

    if (push_state_ == INACTIVE){
        if(state_machine_){
            push_state_ = RELOCATE;
            ROS_INFO("(Push) State: RELOCATE");
        } else{
            push_state_ = PUSH;
            ROS_INFO("(Push) State: PUSH");
        }
        cout << endl;
    }
    //push_state_ = PUSH;

    push_active_ = true;
    goal_reached_ = false;
    current_target_ind_ = 0;

    rel_ = false;

}

void PushPlanner::stopPush(){

    push_state_ = INACTIVE;
    push_active_  = false;
}

geometry_msgs::Twist PushPlanner::getControlCommand(){
    geometry_msgs::Twist cmd;

    if (push_state_ == INACTIVE)
        cmd = getNullTwist();

    else if (push_state_ == RELOCATE)
        cmd = relocateVelocities();

    else if (push_state_ == APPROACH)
        cmd = approachVelocities();

    else if (push_state_ == PUSH)
        cmd = getVelocities();

    //velocity limits

    if(fabs(cmd.linear.x) > vel_lin_max_) cmd.linear.x = (cmd.linear.x > 0 ? vel_lin_max_ : - vel_lin_max_);
    if(fabs(cmd.linear.y) > vel_lin_max_) cmd.linear.y = (cmd.linear.y > 0 ? vel_lin_max_ : - vel_lin_max_);
    if(fabs(cmd.angular.z) > vel_ang_max_) cmd.angular.z = (cmd.angular.z > 0 ? vel_ang_max_ : - vel_ang_max_);

    return cmd;
}

geometry_msgs::Twist PushPlanner::relocateVelocities(){
    geometry_msgs::Twist cmd = getNullTwist();

    double attraction_coefficient = 0.6;
    double repulsion_coefficient = 0.6;
    double repulsion_threshold = 1.5 * object_diameter_;
    double rotation_coefficient = 0.8;

    // Attraction
    double G_attr_x = -attraction_coefficient*(pose_robot_.x - relocate_target_(0));
    double G_attr_y = -attraction_coefficient*(pose_robot_.y - relocate_target_(1));

    // Repulsion
    double G_rep_x, G_rep_y;


    if ((dR2O < repulsion_threshold) && (abs(aORP) > 0.3)){
        G_rep_x =  - repulsion_coefficient * (pose_robot_.x - pose_object_.pose.position.x) * (1 / pow(dR2O,2) - repulsion_threshold / pow(dR2O,3));
        G_rep_y =  - repulsion_coefficient * (pose_robot_.y - pose_object_.pose.position.y) * (1 / pow(dR2O,2) - repulsion_threshold / pow(dR2O,3));
    }
    else {
        G_rep_x = 0;
        G_rep_y = 0;
    }

    double G_x = G_attr_x + G_rep_x;
    double G_y = G_attr_y + G_rep_y;

    vec vel_R_  = rotate2DVector(G_x ,G_y, -pose_robot_.theta);
    cmd.linear.x = vel_R_(0);
    cmd.linear.y = vel_R_(1);

    //orientation cmd
    cmd.angular.z = rotation_coefficient * rotationDifference(aR2O, pose_robot_.theta);
    cout<<cmd.angular.z<<endl;
    return cmd;

}

geometry_msgs::Twist PushPlanner::approachVelocities(){
    geometry_msgs::Twist cmd = getNullTwist();

    double attraction_coefficient = 0.6;
    double rotation_coefficient = 0.3;

    // Attraction
    double G_attr_x = -attraction_coefficient*(pose_robot_.x - pose_object_.pose.position.x);
    double G_attr_y = -attraction_coefficient*(pose_robot_.y - pose_object_.pose.position.y);

    vec vel_R_  = rotate2DVector(G_attr_x ,G_attr_y, -pose_robot_.theta);
    cmd.linear.x = vel_R_(0);
    cmd.linear.y = vel_R_(1);

    //orientation cmd
    cmd.angular.z = - rotation_coefficient * rotationDifference(aO2P, pose_robot_.theta);

    return cmd;

}

void PushPlanner::updateMatrix(){
    current_target_vec_(elem_count_, 0) = current_target_.pose.position.x;
    current_target_vec_(elem_count_, 1) = current_target_.pose.position.y;
    current_target_vec_(elem_count_, 2) = tf::getYaw(current_target_.pose.orientation);

    pose_object_vec_(elem_count_, 0) = pose_object_.pose.position.x;
    pose_object_vec_(elem_count_, 1) = pose_object_.pose.position.y;
    pose_object_vec_(elem_count_, 2) = tf::getYaw(pose_object_.pose.orientation);

    pose_robot_vec_(elem_count_, 0) = pose_robot_.x;
    pose_robot_vec_(elem_count_, 1) = pose_robot_.y;
    pose_robot_vec_(elem_count_, 2) = pose_robot_.theta;

    current_time_vec_(elem_count_, 0) = current_time_;

    elem_count_++;

    if (elem_count_ == current_target_vec_.n_rows) {
        current_target_vec_.resize(current_target_vec_.n_rows + pushing_path_.poses.size(), 3);
        pose_robot_vec_.resize(current_target_vec_.n_rows + pushing_path_.poses.size(), 3);
        pose_object_vec_.resize(current_target_vec_.n_rows + pushing_path_.poses.size(), 3);
        current_time_vec_.resize(current_time_vec_.n_rows + pushing_path_.poses.size(), 1);


    }

}

void PushPlanner::saveData(string path){

    std::ofstream rFile, oFile, tarFile, tFile, cFile ;

    string nameF = path  + experimentName + "_robot.txt";
    rFile.open(nameF.c_str());

    string nameO = path  + experimentName + "_object.txt";
    oFile.open(nameO.c_str());

    string nameT = path + experimentName + "_target.txt";
    tarFile.open(nameT.c_str());

    string namet = path  + experimentName + "_time.txt";
    tFile.open(namet.c_str());

    string nameC = path  + experimentName + "_corr.txt";
    cFile.open(nameC.c_str());



    for (int i = 0; i < elem_count_; i ++){
        for(int j = 0; j < 3; j++){
            rFile << pose_robot_vec_(i, j) << "\t";
            oFile << pose_object_vec_(i, j) << "\t";
            tarFile << current_target_vec_(i, j) << "\t";

        }
        tFile << current_time_vec_(i, 0) << "\t"<< endl;
        rFile << endl;
        oFile << endl;
        tarFile << endl;
    }

    for (int i = 0; i < pushing_path_.poses.size() ; i ++){
        cFile << pushing_path_.poses[i].pose.position.x<<"\t"<<pushing_path_.poses[i].pose.position.y<<"\t"<<corridor_width_array_.at(i)<<endl;
    }



    tFile.close();
    rFile.close();
    tarFile.close();
    oFile.close();
    cFile.close();

    this->saveDataChild(path);


}

void PushPlanner::setExperimentName(string name){
    this->experimentName = name;
}

void PushPlanner::setSim(){
    this->sim_=true;
}

geometry_msgs::PoseStamped PushPlanner::getLookaheadPointFixedDistance(geometry_msgs::PoseStamped pose_object_){

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
    //cout<<"min ind "<<p_min_ind <<endl;

    int p_lookahead = 0;
    double neighbourhood_min = 0.05;

    for (size_t i = p_min_ind; i < pushing_path_.poses.size(); i++) {

        double d_curr = distancePoints(pushing_path_.poses[i].pose.position.x, pushing_path_.poses[i].pose.position.y, pushing_path_.poses[p_min_ind].pose.position.x, pushing_path_.poses[p_min_ind].pose.position.y);
        if (abs(d_curr - lookahead_) < neighbourhood_min){
            neighbourhood_min = abs(d_curr - lookahead_);
            p_lookahead = i;
        }
    }
//    if ((distancePoints(pushing_path_.poses[p_min_ind].pose.position.x, pushing_path_.poses[p_min_ind].pose.position.y, pose_object_.pose.position.x, pose_object_.pose.position.y) > 0.05)&& fixed_){
//        p_lookahead = p_min_ind + 1;
//    }
    //if(p_min_ind > pushing_path_.poses.size() - 10 ); p_lookahead = pushing_path_.poses.size() - 1;
    //cout<<"look "<<p_lookahead<<endl;


    return pushing_path_.poses[p_lookahead];

}
