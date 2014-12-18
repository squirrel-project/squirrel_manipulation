#include <ros/ros.h>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <squirrel_object_manipulation/RobotinoControl.hpp>
#include <squirrel_manipulation_msgs/PickUpAction.h>
#include <actionlib/server/simple_action_server.h>

using namespace std;
using namespace ros;

typedef actionlib::SimpleActionServer<squirrel_manipulation_msgs::PickUpAction> Server;

void execute(const squirrel_manipulation_msgs::PickUpGoalConstPtr& goal, Server* as) {
    // Do lots of awesome groundbreaking robot stuff here
    as->setSucceeded();
}

int main(int argc, char** args) {

    ros::init(argc, args, "pickup_server");
    ros::NodeHandle n;
    sleep(0.5);

    Server server(n, "pickup", boost::bind(&execute, _1, &server), false);
    sleep(0.5);

    server.start();
    ros::spin();

    return 0;

}
