#include "../include/squirrel_arm_control/squirrelbasecontrolqueue.hpp"

using namespace std;
using namespace arma;
using namespace kukadu;

SquirrelBaseControlQueue::SquirrelBaseControlQueue(double cycleTime, KUKADU_SHARED_PTR<ros::NodeHandle> node) : ControlQueue(3, cycleTime, KUKADU_SHARED_PTR<Kinematics>()), moveTopic("/cmd_vel"), bumperTopic("/bumper"), poseTopic("/squirrel_localizer_pose") {

    this->node = node;

    currentBumper = false;

    pubMove = node->advertise<geometry_msgs::Twist>(moveTopic, 1);

    subPose = node->subscribe(poseTopic, 1, &SquirrelBaseControlQueue::updatePose, this);
    subBumper = node->subscribe(bumperTopic, 1, &SquirrelBaseControlQueue::callbackBumper, this);

    jointNames.push_back("x_joint");
    jointNames.push_back("y_joint");
    jointNames.push_back("rotation");

}

void SquirrelBaseControlQueue::updatePose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg) {

    poseMutex.lock();
    currentPose = pose_msg->pose.pose;
    poseMutex.unlock();

}


void SquirrelBaseControlQueue::callbackBumper(std_msgs::Bool msg) {
    currentBumper = msg.data;
}

SquirrelBaseControlQueue::~SquirrelBaseControlQueue() {

}

void SquirrelBaseControlQueue::setInitValues() {
    // nothing required
}

void SquirrelBaseControlQueue::jointPtpInternal(arma::vec joints) {
    throw "(SquirrelBaseControlQUeue) joint ptp not supported";
}

void SquirrelBaseControlQueue::submitNextJointMove(arma::vec joints) {
    // senkas smoothing whatever
}

void SquirrelBaseControlQueue::cartPtpInternal(geometry_msgs::Pose pose) {
    // move base
}

void SquirrelBaseControlQueue::submitNextCartMove(geometry_msgs::Pose pose) {

    geometry_msgs::Pose currenPose = getCurrentCartesianPose();

    // senkas smoothing whatever

}

void SquirrelBaseControlQueue::setCurrentControlTypeInternal(int controlType) {
    // ignore
}

void SquirrelBaseControlQueue::setJntPtpThresh(double thresh) {
    throw "(SquirrelBaseControlQueue) joint ptp thresh is not supported";
}

void SquirrelBaseControlQueue::setAdditionalLoad(float loadMass, float loadPos) {
    // ignore
}

void SquirrelBaseControlQueue::setStiffness(float cpstiffnessxyz, float cpstiffnessabc, float cpdamping, float cpmaxdelta, float maxforce, float axismaxdeltatrq) {
    // ignore
}

bool SquirrelBaseControlQueue::stopQueueWhilePtp() {
    return true;
}

int SquirrelBaseControlQueue::getCurrentControlType() {
    return 0;
}

geometry_msgs::Pose SquirrelBaseControlQueue::getCurrentCartesianPose() {

    // get the global pose
    poseMutex.lock();
    geometry_msgs::Pose poseCopy = currentPose;
    poseMutex.unlock();
    return poseCopy;

}

mes_result SquirrelBaseControlQueue::getCurrentJoints() {
    mes_result ret;
    geometry_msgs::Pose currentPose = getCurrentCartesianPose();
    ret.joints = createJointsVector(3, currentPose.position.x, currentPose.position.y, tf::getYaw(currentPose.orientation));
    ret.time = getCurrentTime();
    return ret;
}

mes_result SquirrelBaseControlQueue::getCurrentJntFrcTrq() {
    mes_result ret;
    ret.joints = vec(3);
    ret.joints.fill(0);
    ret.time = getCurrentTime();
    return ret;
}

mes_result SquirrelBaseControlQueue::getCurrentCartesianFrcTrq() {
    mes_result ret;
    ret.joints = vec(3);
    ret.joints.fill(0);
    ret.time = getCurrentTime();
    return ret;
}

std::string SquirrelBaseControlQueue::getRobotName() {
    return string("robotino_base");
}

std::string SquirrelBaseControlQueue::getRobotFileName() {
    return string("robotino_base");
}

std::vector<std::string> SquirrelBaseControlQueue::getJointNames() {

    return jointNames;

}

std::vector<arma::vec> SquirrelBaseControlQueue::computeIk(geometry_msgs::Pose targetPose) {
    throw "(SquirrelBaseControlQueue) ik is not supported";
}

geometry_msgs::Pose SquirrelBaseControlQueue::computeFk(std::vector<double> joints) {
    throw "(SquirrelBaseControlQueue) fk is not supported";
}

void SquirrelBaseControlQueue::safelyDestroy() {

}
