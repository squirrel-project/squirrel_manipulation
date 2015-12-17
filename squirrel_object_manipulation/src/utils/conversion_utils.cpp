#include "squirrel_object_manipulation/conversion_utils.hpp"

using namespace std;

geometry_msgs::PoseStamped TransformFrame(geometry_msgs::PoseStamped pose_in, string frame_out){

    geometry_msgs::PoseStamped pose_out;
    tf::TransformListener tf_listener;

    try {
        tf_listener.waitForTransform(pose_in.header.frame_id, frame_out, ros::Time::now(), ros::Duration(0.2));
        tf_listener.transformPose(frame_out, pose_in, pose_out);
    } catch (tf::TransformException& ex) {
        std::string ns = ros::this_node::getNamespace();
        std::string node_name = ros::this_node::getName();
        ROS_ERROR("Push: %s/%s: %s", ns.c_str(), node_name.c_str(), ex.what());

    }

    return pose_out;

}

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

geometry_msgs::PoseStamped tf_stamped2pose_stamped(tf::StampedTransform tf_in){
    geometry_msgs::PoseStamped Emap, Eloc;

    Emap.pose.position.x = tf_in.getOrigin().x();
    Emap.pose.position.y = tf_in.getOrigin().y();
    Emap.pose.position.z = tf_in.getOrigin().z();;
    Emap.pose.orientation.x = tf_in.getRotation().x();
    Emap.pose.orientation.y = tf_in.getRotation().y();
    Emap.pose.orientation.z = tf_in.getRotation().z();;
    Emap.pose.orientation.w = tf_in.getRotation().w();;
    Emap.header.frame_id = tf_in.frame_id_;

    return Emap;

}



double string_to_double(const std::string& s) {
    std::istringstream i(s);
    double x;
    if (!(i >> x))
        return 0;
    return x;
}

bool isQuaternionValid(const geometry_msgs::Quaternion& q){
    // check if the quaternion has nan's or infs
    if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
        return false;
    }

    tf::Quaternion tf_q(q.x, q.y, q.z, q.w);

    // if the length of the quaternion is close to zero
    if(tf_q.length2() < 1e-6){
        return false;
    }

    //check transforms the vertical vector correctly
    tf_q.normalize();

    tf::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if(fabs(dot - 1) > 1e-3){
        return false;
    }

    return true;
}



