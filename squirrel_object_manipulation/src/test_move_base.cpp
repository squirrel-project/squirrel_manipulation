#include <ros/ros.h>
#include <iostream>

#include <squirrel_object_manipulation/RobotinoBaseControl.hpp>


using namespace std;

int main(int argc, char** args) {

    ros::init(argc, args, "base_move_test");
    ros::NodeHandle node;
    usleep(1e6);

    RobotinoBaseControl robotino(node, 20.0, 0.6);
    ros::Rate lRate(20.0);

    double target = robotino.getCurrentState();

    while (ros::ok){

        cout<<"current state "<<robotino.getCurrentState()<<endl;
        target = target - 0.05;
        robotino.move(target);
        lRate.sleep();


    }


}
