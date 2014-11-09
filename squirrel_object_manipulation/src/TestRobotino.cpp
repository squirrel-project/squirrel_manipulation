#include <ros/ros.h>
#include <iostream>

#include "RobotinoControl.hpp"

using namespace std;
using namespace ros;

int main(int argc, char** args) {

    ros::init(argc, args, "robotino");
    ros::NodeHandle node;
    usleep(1e6);

    RobotinoControl robotino(node);
//    robotino.singleMove(10, 0, 0, 0, 0, 0);
    robotino.moveLinearDistance(1.0, 0, 0);

}
