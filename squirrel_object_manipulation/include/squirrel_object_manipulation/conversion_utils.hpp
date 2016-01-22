#ifndef CONVERSIONUTILS
#define CONVERSIONUTILS

#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

geometry_msgs::PoseStamped TransformFrame(geometry_msgs::PoseStamped pose_in, std::string frame_out);
geometry_msgs::PoseStamped Map2Base_link(double x, double y);
geometry_msgs::PoseStamped Base_link2Map(double x, double y);
geometry_msgs::PoseStamped Kinect2Base_link(double x, double y, double z);

geometry_msgs::PoseStamped tf_stamped2pose_stamped(tf::StampedTransform tf_in);

bool isQuaternionValid(const geometry_msgs::Quaternion& q);

double string_to_double(const std::string& s);

geometry_msgs::Twist getNullTwist();

#endif
