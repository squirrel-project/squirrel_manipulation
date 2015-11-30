#ifndef CONVERSIONUTILS
#define CONVERSIONUTILS

#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

geometry_msgs::PoseStamped Map2Base_link(double x, double y);
geometry_msgs::PoseStamped Base_link2Map(double x, double y);
geometry_msgs::PoseStamped Kinect2Base_link(double x, double y, double z);

double string_to_double(const std::string& s);

#endif
