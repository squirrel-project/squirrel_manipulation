#include <uibk_arm_controller/arm_controller.hpp>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <memory>
#include <thread>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>

#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

using namespace std;
using namespace uibk_arm_controller;

bool runController = true;
std::shared_ptr<Arm> robotinoArm;
std::shared_ptr<std::thread> armThread;

void stopHandler(int s){

    runController = false;
    robotinoArm->shutdown();
    armThread->join();

}


vector<double> transformVector(vector<int> v);
vector<double> computeDerivative(vector<int> v1, vector<int> v2, double timeStep);
vector<double> computeDerivative(vector<double> v1, vector<double> v2, double timeStep);

int main(int argc, char** args) {

    ros::init(argc, args, "uibk_arm_controller_node"); ros::NodeHandle node; sleep(1);

    struct sigaction sigIntHandler;

   sigIntHandler.sa_handler = stopHandler;
   sigemptyset(&sigIntHandler.sa_mask);
   sigIntHandler.sa_flags = 0;

   sigaction(SIGINT, &sigIntHandler, NULL);

    robotinoArm = std::shared_ptr<Arm>(new Arm({1, 2, 3, 4, 5}, "/dev/ttyArm",
        {std::make_pair<int, int>(-125000, 130000),
        std::make_pair<int, int>(-140000, 185000),
        std::make_pair<int, int>(-150000, 150000),
        std::make_pair<int, int>(-100000, 100000),
        std::make_pair<int, int>(-140000, 140000)}, 2.0, 3000000));

    robotinoArm->initialize();
    armThread = robotinoArm->runArm();

    ros::Publisher statePublisher = node.advertise<sensor_msgs::JointState>("/real/robotino_arm/joint_control/get_state", 1);

    sensor_msgs::JointState jointStateMsg;
    auto prevPos = transformVector(robotinoArm->getCurrentJointState());
    vector<double> prevVel; for(int i = 0; i < robotinoArm->getDegOfFreedom(); ++i) prevVel.push_back(0.0);
    ros::Rate r(robotinoArm->getFrequency());
    double stepTime = 1.0 / robotinoArm->getFrequency();
    while(runController) {

        jointStateMsg.position = transformVector(robotinoArm->getCurrentJointState());
        jointStateMsg.velocity = computeDerivative(jointStateMsg.position, prevPos, stepTime);
        jointStateMsg.effort = computeDerivative(jointStateMsg.velocity, prevVel, stepTime);

        statePublisher.publish(jointStateMsg);

        prevPos = jointStateMsg.position;
        prevVel = jointStateMsg.velocity;

        r.sleep();

    }

    return 0;

}

vector<double> transformVector(vector<int> v) {
    vector<double> retVal;
    for(auto val : v)
        retVal.push_back((double) val);
    return retVal;
}

vector<double> computeDerivative(vector<int> v1, vector<int> v2, double timeStep) {
    vector<double> der;
    for(int i = 0; i < v1.size(); ++i)
        der.push_back((v2.at(i) - v1.at(i)) / timeStep);
    return der;
}

vector<double> computeDerivative(vector<double> v1, vector<double> v2, double timeStep) {
    vector<double> der;
    for(int i = 0; i < v1.size(); ++i)
        der.push_back((v2.at(i) - v1.at(i)) / timeStep);
    return der;
}
