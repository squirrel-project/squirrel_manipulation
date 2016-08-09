#include "squirrel_object_manipulation/conversion_utils.hpp"

using namespace std;

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

geometry_msgs::Twist getNullTwist(){

    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;

    return cmd;
}



