
#include <iostream>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <memory>
#include <thread>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <kukadu/kukadu.hpp>
#include <kukadu/kinematics/moveitkinematics.hpp>


using namespace std;
using namespace kukadu;

int main(int argc, char** args){


    ros::init(argc, args, "kinesthetic_teaching_demo");
    ros::NodeHandle node; sleep(1);
    ros::AsyncSpinner spinner(4); spinner.start();

    vector<string> controlledJoints{"base_jointx", "base_jointy", "base_jointz", "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"};
    KUKADU_SHARED_PTR<KukieControlQueue> robotinoQueue = KUKADU_SHARED_PTR<KukieControlQueue>( new KukieControlQueue("real", "robotino", node));
    KUKADU_SHARED_PTR<Kinematics> mvKin =  KUKADU_SHARED_PTR<Kinematics> (new MoveItKinematics(robotinoQueue, node, "robotino", controlledJoints, "arm_link5"));
    robotinoQueue->setKinematics(mvKin);
    robotinoQueue->setPathPlanner(KUKADU_DYNAMIC_POINTER_CAST<MoveItKinematics>(mvKin));
    KUKADU_SHARED_PTR<kukadu::SensorStorage> store =  KUKADU_SHARED_PTR<kukadu::SensorStorage>(new kukadu::SensorStorage({robotinoQueue}, std::vector<KUKADU_SHARED_PTR<GenericHand> >(), 400));
    KUKADU_SHARED_PTR<AutoCompensatingFilter> myFilter =  KUKADU_SHARED_PTR<AutoCompensatingFilter> (new AutoCompensatingFilter(robotinoQueue,mvKin));
    robotinoQueue->setFrcTrqSensorFilter(myFilter);
    KUKADU_SHARED_PTR<CustomKinestheticTeacher> kTeacher = KUKADU_SHARED_PTR<CustomKinestheticTeacher> (new CustomKinestheticTeacher(robotinoQueue,mvKin,store));

    cout << "KUKADU version" << endl;

    kTeacher->init();
    cout << "apply some torques and forces for calibration & press to start teaching" << endl;
    getchar();
    cout << "Teaching started..." << endl;
    kTeacher->startTeaching();
    cout << "press any key to start recording" << endl;
    getchar();
    kTeacher->startRecording();

    cout << "press any key to stop recording & teaching" << endl;
    getchar();
    kTeacher->stopRecording();
    kTeacher->stopTeaching();
    cout << "press to play" << endl;
    getchar();
    string storeDir = resolvePath("/tmp/kukadu_teaching_demo");
    std::shared_ptr<SensorData> res= kukadu::SensorStorage::readStorage(robotinoQueue,storeDir + "/kuka_lwr_real_robotino_0");
    std::vector<arma::vec> jointPlan;

    for (int i =0;i<res->getTimes().size();i++){
        auto temp=armadilloToStdVec(res->getJointPosRow(i));
        jointPlan.push_back(stdToArmadilloVec(temp));

    }
    cout << "going to the starting postion..." << endl;
    //ptp
    auto target=armadilloToStdVec(jointPlan.front());
    auto ptpPlan = mvKin->planJointTrajectory({robotinoQueue->getCurrentJoints().joints,stdToArmadilloVec(target)});
    robotinoQueue->setNextTrajectory(ptpPlan);
    robotinoQueue->synchronizeToQueue(1);
    //ptp done
    //play recording
    cout << "playing recording..." << endl;
    robotinoQueue->setNextTrajectory(jointPlan);
    robotinoQueue->synchronizeToQueue(1);

    cout << "press to end" << endl;
    getchar();
    std::cout << "stopped!" << std::endl;
    kTeacher->quit();




}
