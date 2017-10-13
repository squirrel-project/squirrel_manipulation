#ifndef ROBOTINOCONTROL
#define ROBOTINOCONTROL

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>
#include <std_msgs/Float64.h>
#include <boost/thread.hpp>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_listener.h>

#include <squirrel_object_manipulation/math_utils.hpp>
#include <squirrel_object_manipulation/conversion_utils.hpp>


#define ROBOTINO_MOVE_TOPIC "/cmd_vel"
#define ROBOTINO_BUMPER_TOPIC "/bumper"
#define ROBOTINO_DISTANCE_TOPIC "/distance_sensors"
#define ROBOTINO_JOINTSTATE_TOPIC "/joint_states"
#define ROBOTINO_ODOM_TOPIC "/odom"
#define ROBOTINO_TF_TOPIC "/tf"
#define ROBOTINO_CAMERAINFO_TOPIC "/webcam/camera_info"
#define ROBOTINO_IMAGERAW_TOPIC "/webcam/image_raw"
#define ROBOTINO_IMAGECOMPRESSED_TOPIC "/webcam/image_raw/compressed"
#define TILT_TOPIC "/neck_tilt_controller/command"
#define PAN_TOPIC "/neck_pan_controller/command"

class RobotinoControl {

private:

    ros::Subscriber subBumper;
    ros::Subscriber subDistanceSensors;

    ros::Subscriber subAnalogReadings;
    ros::Subscriber subBhaReadings;
    ros::Subscriber subDigitalReadings;
    ros::Subscriber subDistanceSensorsClearing;
    ros::Subscriber subEncoderReadings;
    ros::Subscriber subGripperState;

    ros::Subscriber subJointStates;

    ros::Subscriber subMotorReadings;
    ros::Subscriber subNorthStar;

    ros::Subscriber subOdometry;

    ros::Subscriber subPowerReadings;
    ros::Subscriber subBhaPressures;

    ros::Subscriber subTf;
    ros::Subscriber subCameraInfo;
    ros::Subscriber subImageRaw;
    ros::Subscriber subImageCompressed;



    ros::Publisher pubMove;
    ros::Publisher tiltPub;
    ros::Publisher panPub;

    ros::Publisher pubDigitalValues;

    ros::ServiceClient reset_odom;


    bool bumper;
    sensor_msgs::PointCloud distances;
    sensor_msgs::JointState joints;
    nav_msgs::Odometry odometry;
    tf::tfMessage tf;
    sensor_msgs::CameraInfo cameraInfo;
    sensor_msgs::Image imageRaw;
    sensor_msgs::CompressedImage imageCompressed;

    boost::thread* move_base_thread_;
    bool start_move_base_;
    bool stop_move_base_;

    double spinRateVal;
    geometry_msgs::Twist current_base_vel_;

    void callbackBumper(std_msgs::Bool msg);
    void callbackDistance(sensor_msgs::PointCloud msg);
    void callbackJointState(sensor_msgs::JointState msg);
    void callbackOdometry(nav_msgs::Odometry msg);
    void callbackTf(tf::tfMessage msg);
    void callbackCameraInfo(sensor_msgs::CameraInfo msg);
    void callbackImageRaw(sensor_msgs::Image msg);
    void callbackImageCompressed(sensor_msgs::CompressedImage msg);

    void reset_odometry();

    void moveBaseThread();




public:

    RobotinoControl(ros::NodeHandle& node);
    ~RobotinoControl();

    void singleMove(geometry_msgs::Twist twist);
    void singleMove(double x, double y, double z, double xRot, double yRot, double zRot);

    void moveLinearDistance(double x, double y, double z);
    void rotateDistance(double rot);
    void rotateDistanceTime(double rot);
    void rotateAngle(double rot);
    void move2Pose(geometry_msgs::Pose2D target);

    int sign(double d);

    void moveFwd(double speed);
    void moveFwd();
    void turnRobot(double val);
    void stopRobot();
    bool checkDistances(double maxDist);
    bool checkDistancesPush(double maxDist);
    nav_msgs::Odometry getOdom();

    void setControlFrequency(double val);
    void startMoveBase();
    void stopMoveBase();
    void setZeroBaseVelocity();
    void setBaseVelocity(geometry_msgs::Twist twist);

    void moveTilt(double val);
    void movePan(double val);

};

#endif
