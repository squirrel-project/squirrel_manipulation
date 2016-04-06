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

PushPlanner::PushPlanner(string local_frame_, string global_frame_, geometry_msgs::Pose2D pose_robot_, geometry_msgs::PoseStamped pose_object_, nav_msgs::Path pushing_path_, double lookahead_, double goal_toll_, bool state_machine_, double controller_frequency_, double object_diameter_, double robot_diameter_, double corridor_width_):
    private_nh("~")
{
    this->initialize(local_frame_, global_frame_, pose_robot_, pose_object_, pushing_path_, lookahead_, goal_toll_, state_machine_, controller_frequency_, object_diameter_, robot_diameter_, corridor_width_);
    
}

void PushPlanner::initialize(string local_frame_, string global_frame_, geometry_msgs::Pose2D pose_robot_, geometry_msgs::PoseStamped pose_object_, nav_msgs::Path pushing_path_, double lookahead_, double goal_toll_, bool state_machine_, double controller_frequency_, double object_diameter_, double robot_diameter_, double corridor_width_){
    
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
    
    vis_corridor_ = nh.advertise<visualization_msgs::MarkerArray>("/push_action/push_corridor", 100, true);
    marker_target_c_ = nh.advertise<visualization_msgs::Marker>("/push_action/current_target", 100, true);
    marker_object_c_ = nh.advertise<visualization_msgs::Marker>("/push_action/current_object_pose", 100, true);
    marker_robot_c_ = nh.advertise<visualization_msgs::Marker>("/push_action/current_robot_pose", 100, true);
    marker_point_ = nh.advertise<visualization_msgs::Marker>("/push_action/point", 100, true);
    pushing_plan_pub_ = nh.advertise<nav_msgs::Path>("/push_action/pushing_path", 1000, true);
    
    
    visualise_ = false;
    
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
    this->current_target_ = this->getLookaheadPointDynamic();
    this->current_time_ = ros::Time::now().toSec();
    
    if (visualise_){
        publishMarkerTargetCurrent(current_target_);
        publishMarkerObjectCurrent(pose_object_);
        publishMarkerRobotCurrent(pose_robot_);
        publishCorridor();
        pushing_plan_pub_.publish(pushing_path_);
        
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
            relocate_target_vec_.set_size(3,1);
            relocate_target_.set_size(3);
            rel_ = true;
        }
        
        relocate_target_vec_(span(0,1),relocate_target_vec_.n_cols - 1) = reflectPointOverPoint(pose_object_.pose.position.x, pose_object_.pose.position.y,  current_target_.pose.position.x, current_target_.pose.position.y);
        relocate_target_vec_(2, relocate_target_vec_.n_cols - 1) = aO2P;
        for (int i = 0; i < 3; i ++) relocate_target_(i) = mean(relocate_target_vec_.row(i));
        relocate_target_vec_.resize(3,relocate_target_vec_.n_cols + 1);
        
        if (visualise_)publishPoint(relocate_target_);
        
        if ((distancePoints(pose_robot_.x, pose_robot_.y, relocate_target_ (0), relocate_target_ (1)) < 0.06) && (rotationDifference(relocate_target_(2), pose_robot_.theta) < 0.1) && (dR2O < 2 * object_diameter_)){
            push_state_ = PUSH;
            ROS_INFO("(Push) State: PUSH");
            cout << endl;
        }
        
        if((fabs(aO2P - aR2O) < 0.3) && (fabs(rotationDifference(aR2O, pose_robot_.theta)) < 0.3)){
            push_state_ = APPROACH;
            ROS_INFO("(Push) State: APPROACH");
            cout << endl;
        }
        
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
    double P = distancePoints(pushing_path_.poses[p_min_ind ].pose.position.x, pushing_path_.poses[p_min_ind ].pose.position.y,  pose_object_.pose.position.x, pose_object_.pose.position.y);
    if (P > corridor_width_ / 2) cout <<" exceded corridor"<<endl;
    double D = distancePoints(pushing_path_.poses[pushing_path_.poses.size() - 1].pose.position.x, pushing_path_.poses[pushing_path_.poses.size() - 1].pose.position.y,  pose_object_.pose.position.x, pose_object_.pose.position.y);
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

            double d_min = std::numeric_limits<double>::infinity();
           // double angle_tail = abs(rotationDifference(getVectorAngle(pushing_path_.poses[p_min_ind].pose.position.x - pushing_path_.poses[p_min_ind-1].pose.position.x, pushing_path_.poses[p_min_ind].pose.position.y - pushing_path_.poses[p_min_ind-1].pose.position.y),  getVectorAngle(pose_object_.pose.position.x - pushing_path_.poses[i].pose.position.x, pose_object_.pose.position.y - pushing_path_.poses[i].pose.position.y)));
           // double tail = (corridor_width_ / 2 - d_min) * sin (angle_tail);
            
           // double penalty_tail = tail - robot_diameter_ - object_diameter_ ;
            double  penalty_tail = 0.1;

            //double penalty_curve = corridor_width_ - 1.1 *(d_max - robot_diameter_);
            double penalty_curve = corridor_width_ / 2 - zeta * (d_max + robot_diameter_);

            
            //cout<<"d "<<d<<" dmax "<<d_max<<endl;
            //cout<<" tail " <<tail<<endl;
            //cout<<" penatly curve "<<penalty_curve<<" penatly_tail "<<penalty_tail<<endl;


            
            double cost_curr = 1 / d + zeta * d_max;
            if ((penalty_curve <= 0)||((penalty_tail <= 0))) cost_curr = std::numeric_limits<double>::infinity();
            if (cost_curr < cost_min){
                cost_min = cost_curr;
                p_lookahead = i;
                
            }
            //cout<< "current cost "<<cost_curr<< " min "<<cost_min<<" point "<<i<<endl<<endl;
            
        }
    }
    
    if (p_lookahead == p_min_ind + 1) return current_target_;
    
    return pushing_path_.poses[p_lookahead];
}

geometry_msgs::PoseStamped PushPlanner::getLookaheadPoint(){
    return this->getLookaheadPoint(this->pose_object_);
}

geometry_msgs::PoseStamped PushPlanner::getLookaheadPointDynamic(){
    return this->getLookaheadPointDynamic(this->pose_object_);
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
    double repulsion_threshold = 2 * object_diameter_;
    double rotation_coefficient = 0.3;
    
    // Attraction
    double G_attr_x = -attraction_coefficient*(pose_robot_.x - relocate_target_(0));
    double G_attr_y = -attraction_coefficient*(pose_robot_.y - relocate_target_(1));
    
    // Repulsion
    double G_rep_x, G_rep_y;
    
    if (dR2O < repulsion_threshold){
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
    
    std::ofstream rFile, oFile, tarFile, tFile ;
    
    string nameF = path  + experimentName + "_robot.txt";
    rFile.open(nameF.c_str());
    
    string nameO = path  + experimentName + "_object.txt";
    oFile.open(nameO.c_str());
    
    string nameT = path + experimentName + "_target.txt";
    tarFile.open(nameT.c_str());
    
    string namet = path  + experimentName + "_time.txt";
    tFile.open(namet.c_str());
    
    
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
    
    tFile.close();
    rFile.close();
    tarFile.close();
    oFile.close();
    
    this->saveDataChild(path);
    
}

void PushPlanner::setExperimentName(string name){
    this->experimentName = name;
}
