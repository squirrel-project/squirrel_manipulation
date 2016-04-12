#include "squirrel_object_manipulation/RobotinoControl.hpp"

using namespace ros;
using namespace std;

RobotinoControl::RobotinoControl(ros::NodeHandle& node) {

    subBumper = node.subscribe(ROBOTINO_BUMPER_TOPIC, 1, &RobotinoControl::callbackBumper, this);
    subDistanceSensors = node.subscribe(ROBOTINO_DISTANCE_TOPIC, 1, &RobotinoControl::callbackDistance, this);
    subJointStates = node.subscribe(ROBOTINO_JOINTSTATE_TOPIC, 1, &RobotinoControl::callbackJointState, this);
    subOdometry = node.subscribe(ROBOTINO_ODOM_TOPIC, 1, &RobotinoControl::callbackOdometry, this);
    subTf = node.subscribe(ROBOTINO_TF_TOPIC, 1, &RobotinoControl::callbackTf, this);
    subCameraInfo = node.subscribe(ROBOTINO_CAMERAINFO_TOPIC, 1, &RobotinoControl::callbackCameraInfo, this);
    subImageRaw = node.subscribe(ROBOTINO_IMAGERAW_TOPIC, 1, &RobotinoControl::callbackImageRaw, this);
    // subImageCompressed = node.subscribe(ROBOTINO_IMAGECOMPRESSED_TOPIC, 1, &RobotinoControl::callbackImageCompressed, this);

    pubMove = node.advertise<geometry_msgs::Twist>(ROBOTINO_MOVE_TOPIC, 1);
    tiltPub = node.advertise<std_msgs::Float64>(TILT_TOPIC, 1);
    panPub = node.advertise<std_msgs::Float64>(PAN_TOPIC, 1);

    move_base_thread_ = new boost::thread(boost::bind(&RobotinoControl::moveBaseThread, this));

    spinRateVal = 10;

    usleep(1e6);

}

RobotinoControl::~RobotinoControl(){

    move_base_thread_->interrupt();
    move_base_thread_->join();

    delete move_base_thread_;
}

void RobotinoControl::callbackBumper(std_msgs::Bool msg) {
    bumper = msg.data;
}

void RobotinoControl::callbackDistance(sensor_msgs::PointCloud msg) {
    distances = msg;
}

void RobotinoControl::callbackJointState(sensor_msgs::JointState msg) {
    joints = msg;
}

void RobotinoControl::callbackOdometry(nav_msgs::Odometry msg) {
    odometry = msg;
}

void RobotinoControl::callbackTf(tf::tfMessage msg) {
    this->tf = msg;
}

void RobotinoControl::callbackCameraInfo(sensor_msgs::CameraInfo msg) {
    cameraInfo = msg;
}

void RobotinoControl::callbackImageRaw(sensor_msgs::Image msg) {
    imageRaw = msg;
}

void RobotinoControl::callbackImageCompressed(sensor_msgs::CompressedImage msg) {
    imageCompressed = msg;
}


void RobotinoControl::singleMove(geometry_msgs::Twist twist) {
    pubMove.publish(twist);
    ros::spinOnce();
}

void RobotinoControl::singleMove(double x, double y, double z, double xRot, double yRot, double zRot) {
    geometry_msgs::Twist t;
    t.angular.x = xRot;
    t.angular.y = yRot;
    t.angular.z = zRot;
    t.linear.x = x;
    t.linear.y = y;
    t.linear.z = z;
    singleMove(t);
}

void RobotinoControl::moveLinearDistance(double x, double y, double z) {

    ros::Rate spinRate(spinRateVal);
    ros::spinOnce();
    nav_msgs::Odometry initOdometry = odometry;
    double initX = initOdometry.pose.pose.position.x; double initY = initOdometry.pose.pose.position.y; double initZ = initOdometry.pose.pose.position.z;
    double xDist = x; double yDist = y; double zDist = z;

    while(sqrt(pow(xDist, 2) + pow(yDist, 2) + pow(zDist, 2)) > 0.05) {

        ros::spinOnce();
        double currentX = odometry.pose.pose.position.x - initX; double currentY = odometry.pose.pose.position.y - initY; double currentZ = odometry.pose.pose.position.z - initZ;
        xDist = x - currentX; yDist = y - currentY; zDist = z - currentZ;
        singleMove(xDist, yDist, zDist, 0, 0, 0);
        spinRate.sleep();

    }

}

int RobotinoControl::sign(double d) {
    return (d > 0) ? 1 : ((d < 0) ? -1 : 0);
}

void RobotinoControl::rotateDistanceTime(double rot) {

    ros::Rate spinRate(spinRateVal);
    double timeInterval = 1.0 / spinRateVal;
    double currentTime = 0.0;
    double vel = 0.5;

    /*
    1.07 = k * 2 * 3.1415 + d
    1.00 = k * 0.25 * 3.1415 + d

    0.07 = 1.75 * 3.1415 * k
    k = 0.07 / (1.75 * 3.1415)
    d = 1.07 - k * 2 * 3.1415
    */

    double timeK = 0.07 / (1.75 * 3.1415);
    double timeD = 1.07 - timeK * 2 * 3.1415;
    double timeCoeff = timeK * rot + timeD;
    double rotationTime = timeCoeff * 13.1 / (3.1415 / 4.0) * rot * 0.05 / vel;
    double moveVel = sign(rot) * vel;

    double begin = ros::Time::now().toSec();
    while((ros::Time::now().toSec() - begin) < rotationTime) {
        spinRate.sleep();
        // currentTime += timeInterval;
        singleMove(0, 0, 0, 0, 0, moveVel);
    }

}

// range [-1, +1]
void RobotinoControl::rotateDistance(double rot) {

    if(abs(rot) > 1.0) {
        cout << "function requires in the range [-1, +1]";
        return;
    }

    ros::Rate spinRate(spinRateVal);
    ros::spinOnce();
    nav_msgs::Odometry initOdometry = odometry;
    double initZRot = acos(initOdometry.pose.pose.orientation.z);
    int currentSign = sign(initZRot);
    double prevSign = currentSign;
    double rotDist = rot;
    double ringRestRot = (initZRot + rot) - sign(initZRot + rot) * 1.0;

    cout << initZRot << " " << rot << " " << currentSign << " " << ringRestRot << endl;
    cout << ringRestRot << endl;

    while(abs(rotDist) > 0.05) {

        prevSign = currentSign;

        spinRate.sleep();
        ros::spinOnce();

        double currentRot = 0.0;
        double velRotAdd = 0.0;
        currentSign = sign(odometry.pose.pose.orientation.z);
        if(prevSign == currentSign) {
            //    velRotAdd = ringRestRot;
        } else if(abs(odometry.pose.pose.orientation.z) > 0.3) {
            initZRot = acos(odometry.pose.pose.orientation.z);
            rot = ringRestRot;
        }

        currentRot = acos(odometry.pose.pose.orientation.z) - initZRot;

        rotDist = rot - currentRot;
        singleMove(0, 0, 0, 0, 0, sign(rotDist) * 0.05);


    }

}

void RobotinoControl::rotateAngle(double rot) {
    ros::Rate spinRate(spinRateVal);
    ros::spinOnce();
    geometry_msgs::Pose2D pose_robot_;

    nav_msgs::Odometry Odometry = this->getOdom();
    //robotino position in /odom world
    pose_robot_.theta = tf::getYaw(Odometry.pose.pose.orientation);

    double desired_theta_ = pose_robot_.theta + rot;
    double vel_;
    geometry_msgs::Twist cmd;

    while(abs(rotationDifference(desired_theta_, pose_robot_.theta)) > 0.02){
        Odometry = this->getOdom();
        pose_robot_.theta = tf::getYaw(Odometry.pose.pose.orientation);
        vel_ = 2.4 * rotationDifference(desired_theta_, pose_robot_.theta);
        if(abs(vel_) > 0.6) vel_ = sign(vel_) * 0.6;
        cmd = getNullTwist();
        cmd.angular.z = vel_;
        singleMove(cmd);
        spinRate.sleep();

    }

}


void  RobotinoControl::moveFwd(double speed){

    geometry_msgs::Twist twist;
    twist.linear.z=0;
    twist.linear.x=speed;
    twist.linear.y=0;
    twist.angular.z=0;
    twist.angular.x=0;
    twist.angular.y=0;
    singleMove(twist);
}

void  RobotinoControl::moveFwd(){
    moveFwd(0.2);
}

void RobotinoControl::stopRobot(){
    moveFwd(0.0);
}

void RobotinoControl::turnRobot(double val){
    geometry_msgs::Twist twist;
    twist.linear.z=0;
    twist.linear.x=0;
    twist.linear.y=0;
    twist.angular.z=val;
    twist.angular.x=0;
    twist.angular.y=0;
    singleMove(twist);
}

bool RobotinoControl::checkDistances(double maxDist) {

    if(distances.points.size() > 0){
        geometry_msgs::Point32 currentFrontDistance = distances.points.at(0);
        if(sqrt(pow(currentFrontDistance.x, 2) + pow(currentFrontDistance.y, 2)) < maxDist)
            return false;

        currentFrontDistance = distances.points.at(1);
        if(sqrt(pow(currentFrontDistance.x, 2) + pow(currentFrontDistance.y, 2)) < maxDist)
            return false;

        currentFrontDistance = distances.points.at(2);
        if(sqrt(pow(currentFrontDistance.x, 2) + pow(currentFrontDistance.y, 2)) < maxDist)
            return false;

        currentFrontDistance = distances.points.at(3);
        if(sqrt(pow(currentFrontDistance.x, 2) + pow(currentFrontDistance.y, 2)) < maxDist)
            return false;

        currentFrontDistance = distances.points.at(4);
        if(sqrt(pow(currentFrontDistance.x, 2) + pow(currentFrontDistance.y, 2)) < maxDist)
            return false;

        currentFrontDistance = distances.points.at(5);
        if(sqrt(pow(currentFrontDistance.x, 2) + pow(currentFrontDistance.y, 2)) < maxDist)
            return false;

        currentFrontDistance = distances.points.at(6);
        if(sqrt(pow(currentFrontDistance.x, 2) + pow(currentFrontDistance.y, 2)) < maxDist)
            return false;

        currentFrontDistance = distances.points.at(7);
        if(sqrt(pow(currentFrontDistance.x, 2) + pow(currentFrontDistance.y, 2)) < maxDist)
            return false;

        currentFrontDistance = distances.points.at(8);
        if(sqrt(pow(currentFrontDistance.x, 2) + pow(currentFrontDistance.y, 2)) < maxDist)
            return false;
    }

    return true;

}
bool RobotinoControl::checkDistancesPush(double maxDist) {

    if(distances.points.size() > 0){



        geometry_msgs::Point32 currentFrontDistance = distances.points.at(2);

        if(sqrt(pow(currentFrontDistance.x, 2) + pow(currentFrontDistance.y, 2)) < maxDist+0.23)
            return false;


        currentFrontDistance = distances.points.at(3);
        if(sqrt(pow(currentFrontDistance.x, 2) + pow(currentFrontDistance.y, 2)) < maxDist+0.23)
            return false;

        currentFrontDistance = distances.points.at(4);
        if(sqrt(pow(currentFrontDistance.x, 2) + pow(currentFrontDistance.y, 2)) < maxDist+0.23)
            return false;

        currentFrontDistance = distances.points.at(5);
        if(sqrt(pow(currentFrontDistance.x, 2) + pow(currentFrontDistance.y, 2)) < maxDist+0.23)
            return false;

        currentFrontDistance = distances.points.at(6);
        if(sqrt(pow(currentFrontDistance.x, 2) + pow(currentFrontDistance.y, 2)) < maxDist+0.23)
            return false;

        currentFrontDistance = distances.points.at(7);
        if(sqrt(pow(currentFrontDistance.x, 2) + pow(currentFrontDistance.y, 2)) < maxDist+0.23)
            return false;


    }

    return true;

}
nav_msgs::Odometry RobotinoControl::getOdom(){
    return odometry;
}

void RobotinoControl::reset_odometry(){
}

void RobotinoControl::moveTilt(double val){
    std_msgs::Float64 tilt_msg;
    tilt_msg.data = val;
    tiltPub.publish(tilt_msg);
    ros::spinOnce();
}

void RobotinoControl::movePan(double val){
    std_msgs::Float64 pan_msg;
    pan_msg.data = val;
    panPub.publish(pan_msg);
    ros::spinOnce();
}
void RobotinoControl::moveBaseThread(){

    stop_move_base_= true;

    ros::Rate moveBaseRate(spinRateVal);
    ros::spinOnce();

    while (!stop_move_base_ && ros::ok){
        pubMove.publish(current_base_vel_);
        moveBaseRate.sleep();
        ros::spinOnce();
    }

}

void RobotinoControl::startMoveBase(){
    stop_move_base_ = false;
}

void RobotinoControl::stopMoveBase(){
    stop_move_base_ = true;
    setZeroBaseVelocity();
    stopRobot();
}

void RobotinoControl::setZeroBaseVelocity(){
    geometry_msgs::Twist twist;
    twist.linear.z=0;
    twist.linear.x=0;
    twist.linear.y=0;
    twist.angular.z=0;
    twist.angular.x=0;
    twist.angular.y=0;
    this->setBaseVelocity(twist);
}

void RobotinoControl::setBaseVelocity(geometry_msgs::Twist twist){
    current_base_vel_ = twist;
}

void RobotinoControl::setControlFrequency(double val){
    spinRateVal = val;
}
