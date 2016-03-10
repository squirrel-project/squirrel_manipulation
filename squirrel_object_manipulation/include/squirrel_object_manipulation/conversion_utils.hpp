#ifndef CONVERSIONUTILS
#define CONVERSIONUTILS

#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>

geometry_msgs::PoseStamped tf_stamped2pose_stamped(tf::StampedTransform tf_in);

bool isQuaternionValid(const geometry_msgs::Quaternion& q);

double string_to_double(const std::string& s);

geometry_msgs::Twist getNullTwist();

#endif
