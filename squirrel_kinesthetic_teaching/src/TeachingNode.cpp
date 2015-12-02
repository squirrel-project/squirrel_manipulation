#include <ros/ros.h>

#include "squirrel_kinesthetic_teaching/TeachingNode.hpp"

using namespace ros;
using namespace std;

TeachingNode::TeachingNode(const string& name){
    this->name = name;
    this->learning = false;

    start_cmd_subscriber = node.subscribe("squirrel_manipulation/start_teaching", 10, &TeachingNode::start, this);
    stop_cmd_subscriber = node.subscribe("squirrel_manipulation/stop_teaching", 10, &TeachingNode::stop, this);
    
//    arm_joint_publisher = node.advertise<what message>("wherever", 1000);
}


TeachingNode::~TeachingNode(){
    //delete everything
}


void TeachingNode::start(const std_msgs::String::ConstPtr& msg) {
    if (!learning) {
    if (msg->data.compare("start") == 0) {
	ROS_INFO("Changing into teaching mode");
	learning = true;
    } else {
	ROS_ERROR("Unrecognized message: %s", msg->data.c_str());
    }
    } else {
	ROS_ERROR("Already in teaching mode");
    }
}


void TeachingNode::stop(const std_msgs::String::ConstPtr& msg) {
    if (learning) {
	if (msg->data.compare("stop") == 0 && learning) {
	    ROS_INFO("Exiting teaching mode");
	    learning = false;
	} else {
	    ROS_ERROR("Unrecognized message: %s", msg->data.c_str());
	}
    } else {
	ROS_ERROR("Not in teaching mode");
    }
}

string& TeachingNode::getName(){
    return this->name;
}








