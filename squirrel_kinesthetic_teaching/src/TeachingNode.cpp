#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include "squirrel_kinesthetic_teaching/TeachingNode.hpp"

using namespace ros;
using namespace std;


const double TeachingNode::pause = 10;
const double TeachingNode::delta = 0.01;

TeachingNode::TeachingNode(const string& name){
    this->name = name;
    learning = false;

    start_cmd_subscriber = node.subscribe<std_msgs::String>("squirrel_manipulation/start_teaching", 10, &TeachingNode::start, this);
    stop_cmd_subscriber = node.subscribe<std_msgs::String>("squirrel_manipulation/stop_teaching", 10, &TeachingNode::stop, this);
    ft_wrist_subscriber = node.subscribe<std_msgs::Float64MultiArray>("wrist", 10, &TeachingNode::wristFeedback, this);
    
    loop_rate = new Rate(pause);
    //ft_data = new std_msgs::Float64MultiArray();
}


TeachingNode::~TeachingNode(){
    if(loop_rate != NULL) {
	delete loop_rate;
    }

    if(ft_data != NULL) {
	delete ft_data;
    }
}


void TeachingNode::start(const std_msgs::String::ConstPtr& msg) {
    AsyncSpinner spinner(1);
    if (!learning) {
	if (msg->data.compare("start") == 0) {
	    spinner.start();
	    ROS_INFO("Changing into teaching mode");
	    learning = true;

	    moveit::planning_interface::MoveGroup group("arm");
	    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  
	    
	    while(ros::ok() && learning) {
		if(ft_data != NULL) {
		    geometry_msgs::PoseStamped currentPose = group.getCurrentPose();
	
		    geometry_msgs::Pose target_pose;
		    target_pose.position.x = currentPose.pose.position.x + delta * (*ft_data)[0];
		    target_pose.position.y = currentPose.pose.position.y + delta * (*ft_data)[1];
		    target_pose.position.z = currentPose.pose.position.z + delta * (*ft_data)[2];
		    //target_pose.orientation.w = currentPose.pose.orientation.w + delta * (*ft_data)[3];
		    //target_pose.orientation.x = currentPose.pose.orientation.x + delta * (*ft_data)[4];
		    //target_pose.orientation.y = currentPose.pose.orientation.y + delta * (*ft_data)[5];
		    //target_pose.orientation.z = currentPose.pose.orientation.z + delta * (*ft_data)[6];

		    group.setPoseTarget(target_pose);
		
		    moveit::planning_interface::MoveGroup::Plan my_plan;
		    bool success = group.plan(my_plan);

		    group.move();
		}
		
		spinOnce();
		loop_rate->sleep();
	    }
	} else {
	    ROS_ERROR("Unrecognized command: %s", msg->data.c_str());
	}
    } else {
	spinner.stop();
	ROS_ERROR("Already in teaching mode");
    }
}


void TeachingNode::stop(const std_msgs::String::ConstPtr& msg) {
    if (learning) {
	if (msg->data.compare("stop") == 0 && learning) {
	    ROS_INFO("Exiting teaching mode");
	    learning = false;
	} else {
	    ROS_ERROR("Unrecognized command: %s", msg->data.c_str());
	}
    } else {
	ROS_ERROR("Not in teaching mode");
    }
}


void TeachingNode::wristFeedback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if(ft_data != NULL) {
	delete ft_data;
    }
    ft_data = new vector<double>(msg->data.size());
    ft_data = &msg->data;
}


string& TeachingNode::getName(){
    return name;
}








