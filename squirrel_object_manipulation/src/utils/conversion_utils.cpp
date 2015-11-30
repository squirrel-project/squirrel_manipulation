#include "squirrel_object_manipulation/conversion_utils.hpp"

using namespace std;

geometry_msgs::PoseStamped Map2Base_link(double x, double y){

    geometry_msgs::PoseStamped Emap, Eloc;
    tf::TransformListener tf_listener;

    Emap.pose.position.x=x;
    Emap.pose.position.y=y;
    Emap.pose.position.z=0;
    Emap.pose.orientation.x=0;
    Emap.pose.orientation.y=0;
    Emap.pose.orientation.z=0;
    Emap.pose.orientation.w=1;
    Emap.header.frame_id="/map";
    try {
        tf_listener.waitForTransform("/map","/base_link", ros::Time::now(), ros::Duration(0.2));
        tf_listener.transformPose("/base_link",Emap,Eloc);
    } catch (tf::TransformException& ex) {
        std::string ns = ros::this_node::getNamespace();
        std::string node_name = ros::this_node::getName();
        ROS_ERROR("Push: %s/%s: %s", ns.c_str(), node_name.c_str(), ex.what());

    }

    return Eloc;
}

geometry_msgs::PoseStamped Kinect2Base_link(double x, double y, double z){

    geometry_msgs::PoseStamped Ekin, Eloc;
    tf::TransformListener tf_listener;

    Ekin.pose.position.x=x;
    Ekin.pose.position.y=y;
    Ekin.pose.position.z=z;
    Ekin.pose.orientation.x=0;
    Ekin.pose.orientation.y=0;
    Ekin.pose.orientation.z=0;
    Ekin.pose.orientation.w=1;
    Ekin.header.frame_id="/kinect_rgb_optical_frame";
    try {
        tf_listener.waitForTransform("/kinect_rgb_optical_frame","/base_link", ros::Time::now(), ros::Duration(0.2));
        tf_listener.transformPose("/base_link",Ekin,Eloc);
    } catch (tf::TransformException& ex) {
        std::string ns = ros::this_node::getNamespace();
        std::string node_name = ros::this_node::getName();
        ROS_ERROR("Push: %s/%s: %s", ns.c_str(), node_name.c_str(), ex.what());

    }

    return Eloc;
}

geometry_msgs::PoseStamped Base_link2Map(double x, double y){

    geometry_msgs::PoseStamped Emap, Eloc;
    tf::TransformListener tf_listener;

    Emap.pose.position.x=x;
    Emap.pose.position.y=y;
    Emap.pose.position.z=0;
    Emap.pose.orientation.x=0;
    Emap.pose.orientation.y=0;
    Emap.pose.orientation.z=0;
    Emap.pose.orientation.w=1;
    Emap.header.frame_id="/base_link";
    try {
        tf_listener.waitForTransform("/base_link","/map", ros::Time::now(), ros::Duration(0.2));
        tf_listener.transformPose("/map",Emap,Eloc);
    } catch (tf::TransformException& ex) {
        std::string ns = ros::this_node::getNamespace();
        std::string node_name = ros::this_node::getName();
        ROS_ERROR("%s/%s: %s", ns.c_str(), node_name.c_str(), ex.what());

    }

    return Eloc;
}

double string_to_double(const std::string& s) {
    std::istringstream i(s);
    double x;
    if (!(i >> x))
        return 0;
    return x;
}

