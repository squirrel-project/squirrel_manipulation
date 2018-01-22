#include "../include/PushPlanner.hpp"

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
    marker.color.b = 1.0;
    marker_target_c_.publish(marker);
}

void PushPlanner::publishMarkerObjectCurrent(geometry_msgs::PoseStamped t_pose) {

    visualization_msgs::Marker marker;
    marker.header.frame_id = t_pose.header.frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "push_action";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = t_pose.pose;
    marker.scale.x = object_diameter_ ;
    marker.scale.y = object_diameter_ ;
    marker.scale.z = object_diameter_ ;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker_object_c_.publish(marker);
}

void PushPlanner::publishMarkerRobotCurrent(geometry_msgs::Pose2D t_pose) {

    visualization_msgs::Marker marker;
    marker.header.frame_id = global_frame_;
    marker.header.stamp = ros::Time();
    marker.ns = "push_action";
    marker.id = 2;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = t_pose.x;
    marker.pose.position.y = t_pose.y;
    marker.pose.position.z = 0;
    marker.scale.x = robot_diameter_;
    marker.scale.y = robot_diameter_ ;
    marker.scale.z = 0.3;
    marker.color.a = 1.0;
    marker.color.r = 0.8;
    marker.color.g = 0.8;
    marker.color.b = 0.8;
    marker_robot_c_.publish(marker);
}

void PushPlanner::publishPoint(geometry_msgs::PoseStamped t_pose) {

    visualization_msgs::Marker marker;
    marker.header.frame_id = t_pose.header.frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "push_action";
    marker.id = 3;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = t_pose.pose;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker_point_.publish(marker);
}
void PushPlanner::publishPoint(geometry_msgs::PoseStamped t_pose, int id) {

    visualization_msgs::Marker marker;
    marker.header.frame_id = t_pose.header.frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "push_action_" + NumberToString(id);
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = t_pose.pose;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
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

void PushPlanner::publishCorridor(){
    visualization_msgs::MarkerArray marker_array;

    for(size_t i = 0; i < pushing_path_.poses.size(); i++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = pushing_path_.header.frame_id;
        marker.header.stamp = ros::Time();
        marker.ns = "push_corridor";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::MODIFY;
        marker.pose = pushing_path_.poses[i].pose;
        marker.pose.position.z= -0.25;
        if(corridor_width_ > 0.0){
            marker.scale.x = corridor_width_ ;
            marker.scale.y = corridor_width_ ;
        }
        else {
            marker.scale.x = 1.05*corridor_width_array_.at(i);
            marker.scale.y = 1.05*corridor_width_array_.at(i);
        }
        marker.scale.z = 0.05;
        marker.color.a = 0.3;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker_array.markers.push_back(marker);
    }
    vis_corridor_.publish( marker_array );

    visualization_msgs::MarkerArray marker_array_object;

    for(size_t i = 0; i < pushing_path_.poses.size(); i++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = pushing_path_.header.frame_id;
        marker.header.stamp = ros::Time();
        marker.ns = "object_corridor";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::MODIFY;
        marker.pose = pushing_path_.poses[i].pose;
        marker.pose.position.z= -0.05;

        marker.scale.x = 1.05*(corridor_width_array_.at(i) - object_diameter_ - 2*robot_diameter_ > 0 ? corridor_width_array_.at(i) - object_diameter_ - 2*robot_diameter_ : 0);
        marker.scale.y = 1.05*(corridor_width_array_.at(i) - object_diameter_ - 2*robot_diameter_ > 0 ? corridor_width_array_.at(i) - object_diameter_ - 2*robot_diameter_ : 0);
        //marker.scale.x = corridor_object_width_array_.at(i);
        //marker.scale.y = corridor_object_width_array_.at(i);

        marker.scale.z = 0.1;
        marker.color.a = 0.3;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker_array_object.markers.push_back(marker);
         marker_array_object.markers.push_back(marker);

    }
    vis_object_corridor_.publish(marker_array_object);

}

void PushPlanner::deleteMarkers(){
    visualization_msgs::Marker marker;
    marker.action = 2;
    marker.ns = "push_action";
    marker.id = 0;
    marker_target_c_.publish(marker);

    marker.ns = "push_action";
    marker.id = 1;
    marker_object_c_.publish(marker);

    marker.ns = "push_action";
    marker.id = 2;
    marker_robot_c_.publish(marker);

    marker.ns = "push_action";
    marker.id = 3;
    marker_point_.publish(marker);

    visualization_msgs::MarkerArray marker_array, marker_array_object;
    for(size_t i = 0; i < pushing_path_.poses.size(); i++) {
        marker.ns = "push_corridor";
        marker.id = i;
        marker_array.markers.push_back(marker);
        marker.ns = "object_corridor";
        marker_array_object.markers.push_back(marker);

    }
    vis_corridor_.publish( marker_array);
    vis_object_corridor_.publish(marker_array_object);

    pushing_path_.poses.clear();
    pushing_plan_pub_.publish(pushing_path_);
    pushing_edge_p_pub_.publish(pushing_path_);
    pushing_edge_n_pub_.publish(pushing_path_);
    object_edge_p_pub_.publish(pushing_path_);
    object_edge_n_pub_.publish(pushing_path_);


}

