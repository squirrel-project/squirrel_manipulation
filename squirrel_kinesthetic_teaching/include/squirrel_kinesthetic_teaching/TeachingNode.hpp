#ifndef TEACHING_NODE
#define TEACHING_NODE

#include <string>
#include <ros/ros.h>

#include <std_msgs/String.h>

class TeachingNode {

    private:
        ros::NodeHandle node;
        ros::Subscriber ft_wrist_subscriber;
        ros::Subscriber start_cmd_subscriber;
        ros::Subscriber stop_cmd_subscriber;
        ros::Publisher arm_joint_publisher;

        std::string name;
        bool learning;

    public:
        TeachingNode(const std::string& name);
        ~TeachingNode();

        void start(const std_msgs::String::ConstPtr& msg);   //start kinesthetic teaching
        void stop(const std_msgs::String::ConstPtr& msg);    //stop  kinesthetic teaching

        std::string& getName();
};

#endif
