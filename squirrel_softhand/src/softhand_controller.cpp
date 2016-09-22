#include "ros/ros.h"
#include "squirrel_manipulation_msgs/SoftHandGrasp.h"
#include "stdlib.h"
#include "math.h"
#include "../include/qbmove_communications.h"

int device_id;
comm_settings comm_settings_t;

bool actuate(squirrel_manipulation_msgs::SoftHandGrasp::Request & req,
             squirrel_manipulation_msgs::SoftHandGrasp::Response & res)
{
    short int inputs[2];
    inputs[0] = ceil(req.position*17000.0);
    ROS_INFO("Position: %d", inputs[0]);
    inputs[1] = 0;    
    commSetInputs(&comm_settings_t, device_id, inputs);
    if(comm_settings_t.file_handle == INVALID_HANDLE_VALUE)
    {
        ROS_ERROR("Couldn't move hand!");
        res.success = false;
        return EXIT_FAILURE;
    }
    res.success = true;
    return true;    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "squirrel_softhand");
    ros::NodeHandle n;

    char ports[10][255];
    RS485listPorts(ports);
    std::string port;
    ros::param::param<std::string>("port", port, ports[0]);
    ROS_INFO("Selected port: %s", port.c_str());

    int device_id;
    ros::param::param<int>("device_id", device_id, 1);
    ROS_INFO("Device ID: %d", device_id);

    openRS485(&comm_settings_t, port.c_str());
    if(comm_settings_t.file_handle == INVALID_HANDLE_VALUE)
    {
        ROS_ERROR("Couldn't connect to serial port: %p", port.c_str());
        return EXIT_FAILURE;
    } else {
        ROS_INFO("Connected to %s.", port.c_str());
    }
    
    commActivate(&comm_settings_t, device_id, 1);    
    ros::ServiceServer service = n.advertiseService("softhand_grasp", actuate);
    ROS_INFO("Ready to grasp.");
    
    while(ros::ok()){
        ros::spinOnce();
    }

    commActivate(&comm_settings_t, device_id, 0);
    closeRS485(&comm_settings_t);

    return EXIT_SUCCESS;
}
