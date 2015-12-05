#ifndef TEACHING_NODE
#define TEACHING_NODE

#include <string>
#include <ros/ros.h>
#include <vector>

#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;
using namespace ros;

class TeachingNode {

    private:
        NodeHandle node;
        Subscriber ft_wrist_subscriber;
        Subscriber start_cmd_subscriber;
        Subscriber stop_cmd_subscriber;
        Publisher arm_joint_publisher;

        string name;
        bool learning;
        Rate* loop_rate;
        static const double delta;
        static const double pause;
        const vector<double>* ft_data;


    public:
        TeachingNode(const std::string& name);
        ~TeachingNode();

        void start(const std_msgs::String::ConstPtr& msg);   //start kinesthetic teaching
        void stop(const std_msgs::String::ConstPtr& msg);    //stop  kinesthetic teaching
        void wristFeedback(const std_msgs::Float64MultiArray::ConstPtr& msg);

        std::string& getName();
};

#endif
