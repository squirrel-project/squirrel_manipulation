#include <math.h>
#include <ros/ros.h>
#include <iostream>
#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>

#include "squirrel_object_manipulation/RobotinoControl.hpp"

using namespace std;
using namespace ros;

bool firstSet = false;
tf::tfMessage newTarget;
double robotinoAngle = 0.0;
double robotinoDist = 0.0;

void arCallback(tf::tfMessage msg);

int main(int argc, char** args) {

    ros::init(argc, args, "arTest");
    ros::NodeHandle node;
    usleep(1e6);

    RobotinoControl robotino(node);

//    robotino.rotateDistanceTime(3.1415 * 2);
//    robotino.singleMove(10, 0, 0, 0, 0, 0);
//    robotino.moveLinearDistance(1.0, 0, 0);

    ros::Subscriber markerSub = node.subscribe("arMarker/tf", 1, arCallback);
    ros::Rate lRate(5);

    cout << "everything initialized" << endl;

    while(!firstSet) {
        ros::spinOnce();
        lRate.sleep();
    }

    double prevRobotinoAngle = robotinoAngle;
    double prevRobotinoDist = robotinoDist;
    double minDist = 0.5;

    while(true) {

        /*

        if(abs(prevRobotinoAngle - robotinoAngle) > 0.01) {
            moveAngle = -robotinoAngle * 1.3;
            prevRobotinoAngle = robotinoAngle;
        } else {
            moveAngle = 0.0;
        }

        if(robotinoDist > 0.5) {
            moveDist = (robotinoDist - 0.5) / 10.0;
            prevRobotinoDist = robotinoDist;
        } else
            moveDist = 0.0;

        lRate.sleep();
        ros::spinOnce();

   //     cout << abs(prevRobotinoAngle - robotinoAngle) << " " << prevRobotinoAngle << " " << robotinoAngle << endl;
   //     cout << moveAngle << " " << moveDist << endl;
        robotino.singleMove(0, 0, 0, 0, 0, moveAngle);

        */

        ros::spinOnce();
        // if angle changed
        if(abs(prevRobotinoAngle - robotinoAngle) > 0.01 && abs(prevRobotinoAngle - robotinoAngle) < 2.5) {
            if(robotinoDist < minDist)
                robotinoDist = minDist;
            robotino.singleMove((robotinoDist - minDist) / 5.0, 0, 0, 0, 0, -(robotinoAngle) * 2.0);
//            robotino.singleMove((robotinoDist - 0.5) / 5.0, 0, 0, 0, 0, -robotino.sign(robotinoAngle) * 0.5);
            prevRobotinoAngle = robotinoAngle;
            prevRobotinoDist = robotinoDist;
        }
        lRate.sleep();

    }

}

void arCallback(tf::tfMessage msg) {

    firstSet = true;
    newTarget = msg;
    geometry_msgs::TransformStamped t = msg.transforms.at(0);
    geometry_msgs::Vector3 translation = t.transform.translation;
    geometry_msgs::Quaternion rotation = t.transform.rotation;
    double yaw = tf::getYaw(rotation);

    if(rotation.x == rotation.x) {

        robotinoAngle = M_PI / 2.0 - atan2(translation.z, translation.x);
        robotinoDist = sqrt( pow(translation.x, 2) + pow(translation.z, 2));

        // cout << translation.x << " " << translation.y << " " << translation.z << " " << robotinoAngle << endl;
        // cout << rotation.x << " " << rotation.y << " " << rotation.z << " " << yaw << endl;

    }

}
