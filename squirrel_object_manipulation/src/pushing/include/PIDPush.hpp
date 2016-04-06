#ifndef PIDPUSH_H
#define PIDPUSH_H

#include "PushPlanner.hpp"

#include <geometry_msgs/Pose2D.h>
#include <control_toolbox/pid.h>

class PIDPush : public PushPlanner {

private:
    ros::NodeHandle nh;
    control_toolbox::Pid pid_x_;
    control_toolbox::Pid pid_y_;
    control_toolbox::Pid pid_theta_;
    control_toolbox::Pid pid_xd_;
    control_toolbox::Pid pid_yd_;


    double p_x_, d_x_, i_x_, i_x_min_, i_x_max_;
    double p_y_, d_y_, i_y_, i_y_min_, i_y_max_;
    double p_theta_, d_theta_, i_theta_, i_theta_min_, i_theta_max_;

protected:
    void initChild();
    void updateChild(){}


public:

    PIDPush();
    geometry_msgs::Twist getVelocities();


};


#endif // PIDPUSH_H
