#include <iostream>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64MultiArray.h>

#define SENSOR_TOPIC "/wrist"
#define KUKADU_SENSOR_TOPIC"/real/robotino/sensoring/cartesian_wrench"

class Listener{

public:
    Listener(){
        start=false;
    }

    geometry_msgs::Wrench current;
    bool start;
    void handler(std_msgs::Float64MultiArray msg){
        start=true;
        geometry_msgs::Vector3 force;
        geometry_msgs::Vector3 torque;

        current.force.x=msg.data.at(0);
        current.force.y=msg.data.at(1);
        current.force.z=msg.data.at(2);

        current.torque.x=msg.data.at(3);
        current.torque.y=msg.data.at(4);
        current.torque.z=msg.data.at(5);
    }

};

Listener myListener;

int main(int argc, char** args){

    ros::init(argc, args, "forceTorqueSensorTopicConversion_node");
    ros::NodeHandle node; sleep(1);

    ros::Subscriber mySubsriber = node.subscribe(SENSOR_TOPIC,1,&Listener::handler,&myListener);
    ros::Publisher myPublisher = node.advertise<geometry_msgs::Wrench>(KUKADU_SENSOR_TOPIC, 1);

    ros::Rate myRate(80);

    while(ros::ok){
    if(myListener.start)
        myPublisher.publish(myListener.current);
    myRate.sleep();
    ros::spinOnce();
    }





}
