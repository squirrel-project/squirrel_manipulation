#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <squirrel_manipulation_msgs/PtpAction.h>
#include <squirrel_manipulation_msgs/JointPtpAction.h>
#include <kukadu/kinematics/komoplanner.hpp>

#include <vector>
#include <memory>
#include <iostream>
#include <kukadu/kukadu.hpp>
#include <cmath>
#include <mutex>

using namespace std;
using namespace arma;
using namespace kukadu;

class PtpAction {
protected:

  bool cartPtpRunning;
  bool jointPtpRunning;
  std::mutex ptpMutexRunning;

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<squirrel_manipulation_msgs::PtpAction> cartPtpAs_;
  actionlib::SimpleActionServer<squirrel_manipulation_msgs::JointPtpAction> jointPtpAs_;
  std::string action_name_;
  std::string jointPtpName;
  std::string tip_link_;
  // create messages that are used to published feedback/result
  squirrel_manipulation_msgs::PtpFeedback cptpfeedback_;
  squirrel_manipulation_msgs::PtpResult cptpresult_;

  squirrel_manipulation_msgs::JointPtpFeedback jptpfeedback_;
  squirrel_manipulation_msgs::JointPtpResult jptpresult_;

  KUKADU_SHARED_PTR<KukieControlQueue> robotinoQueue;
  KUKADU_SHARED_PTR<MoveItKinematics> mvKin;

  KUKADU_SHARED_PTR<kukadu_thread> realLqThread;

public:

  PtpAction(std::string name, std::string jointPtpName, std::string tip_link) :
    cartPtpAs_(nh_, name, boost::bind(&PtpAction::executeCB, this, _1), false),
    jointPtpAs_(nh_, jointPtpName, boost::bind(&PtpAction::executeJointPtp, this, _1), false),
    action_name_(name) {

    this->jointPtpName = jointPtpName;

    cartPtpRunning = false;
    jointPtpRunning = false;

    this->tip_link_ = tip_link;

    //ROS_INFO("setting up control queue");
    //robotinoQueue = KUKADU_SHARED_PTR<KukieControlQueue>(new KukieControlQueue("real", "robotino", nh_));

    //ROS_INFO("creating moveit kinematics instance");
    //vector<string> controlledJoints{"base_jointx", "base_jointy", "base_jointz", "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"};
    //mvKin = make_shared<MoveItKinematics>(robotinoQueue, nh_, "robotino", controlledJoints, "arm_link5", false, 10, 2.0);
    //mvKin = make_shared<MoveItKinematics>(robotinoQueue, nh_, "robotino", controlledJoints, tip_link, false, 4, 2.0);


    //auto komoAll = make_shared<KomoPlanner>(robotinoQueue,
    //		"/home/rss2017/catkin_ws/src/kukadu/external/komo/share/data/robotino_model/robotino.kvg",
    //		"/home/rss2017/catkin_ws/src/kukadu/external/komo/share/data/robotino_model/MT.cfg",
    //	        "");


    //robotinoQueue->setKinematics(mvKin);
    //robotinoQueue->setPathPlanner(mvKin);
    //robotinoQueue->setPathPlanner(komoAll);
    //robotinoQueue->setKinematics(komoAll);

    //ROS_INFO("starting queue");
    //realLqThread = robotinoQueue->startQueue();

    cartPtpAs_.start();
    jointPtpAs_.start();

  }

  ~PtpAction(void) {
  }

  void executeJointPtp(const squirrel_manipulation_msgs::JointPtpGoalConstPtr& goal) {

    ROS_INFO("setting up control queue");
    robotinoQueue = KUKADU_SHARED_PTR<KukieControlQueue>(new KukieControlQueue("real", "robotino", nh_));

    ROS_INFO("creating moveit kinematics instance");
    vector<string> controlledJoints{"base_jointx", "base_jointy", "base_jointz", "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"};
    //mvKin = make_shared<MoveItKinematics>(robotinoQueue, nh_, "robotino", controlledJoints, "arm_link5", false, 10, 2.0);
    mvKin = make_shared<MoveItKinematics>(robotinoQueue, nh_, "robotino", controlledJoints, this->tip_link_, false, 4, 2.0);
    robotinoQueue->setKinematics(mvKin);
    robotinoQueue->setPathPlanner(mvKin);
    ROS_INFO("starting queue");
    realLqThread = robotinoQueue->startQueue();


    jointPtpRunning = true;

    if(!cartPtpRunning) {
      if(robotinoQueue->getCurrentMode() != KukieControlQueue::KUKA_JNT_POS_MODE) {
        robotinoQueue->stopCurrentMode();
        robotinoQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);
      }
      jptpfeedback_.current_status = "executing";
      jointPtpAs_.publishFeedback(jptpfeedback_);

      bool succeded=false;
      try{
	      robotinoQueue->jointPtp(goal->joints.data);
        succeded=true;
      } catch (KukaduException& ex) {
	      jointPtpRunning=false;
        ROS_ERROR("PTP server failed: %s", ex.what());
      }
      robotinoQueue->stopCurrentMode();
      if(succeded) {
	      jptpresult_.result_status = "execution done";
        jointPtpAs_.setSucceeded(jptpresult_);
      } else {
        jptpresult_.result_status = "execution failed";
        jointPtpAs_.setSucceeded(jptpresult_);
      }
    } else {

      jptpresult_.result_status = "execution failed because another ptp is currently running";
      jointPtpAs_.setSucceeded(jptpresult_);

    }

    jointPtpRunning = false;

    robotinoQueue->stopQueue();

  }

  void executeCB(const squirrel_manipulation_msgs::PtpGoalConstPtr &goal) {

    ROS_INFO("setting up control queue");
    robotinoQueue = KUKADU_SHARED_PTR<KukieControlQueue>(new KukieControlQueue("real", "robotino", nh_));

    ROS_INFO("creating moveit kinematics instance");
    vector<string> controlledJoints{"base_jointx", "base_jointy", "base_jointz", "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"};
    //mvKin = make_shared<MoveItKinematics>(robotinoQueue, nh_, "robotino", controlledJoints, "arm_link5", false, 10, 2.0);
    mvKin = make_shared<MoveItKinematics>(robotinoQueue, nh_, "robotino", controlledJoints, this->tip_link_, false, 4, 2.0);
    robotinoQueue->setKinematics(mvKin);
    robotinoQueue->setPathPlanner(mvKin);
    ROS_INFO("starting queue");
    realLqThread = robotinoQueue->startQueue();

    cartPtpRunning = true;

    if(!jointPtpRunning) {
      if(robotinoQueue->getCurrentMode() != KukieControlQueue::KUKA_JNT_POS_MODE) {
        robotinoQueue->stopCurrentMode();
        robotinoQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);
      }
      cptpfeedback_.current_status = "executing";
      cartPtpAs_.publishFeedback(cptpfeedback_);

      bool succeded=false;
      try{
        robotinoQueue->cartesianPtp(goal->pose);
        succeded=true;
      } catch (KukaduException& ex) {
	cartPtpRunning = false;
	ROS_ERROR("PTP Server failed: %s", ex.what());
      }
      robotinoQueue->stopCurrentMode();
      if (succeded) {
        cptpresult_.result_status = "execution done.";
	cartPtpAs_.setSucceeded(cptpresult_);
      } else {
        cptpresult_.result_status = "execution failed.";
	cartPtpAs_.setAborted(cptpresult_);
      }
    } else {
      cptpresult_.result_status = "execution failed because another ptp is currently running";
      cartPtpAs_.setSucceeded(cptpresult_);
    }

    cartPtpRunning = false;

    robotinoQueue->stopQueue();

  }

};


int main(int argc, char** argv) {

  ros::init(argc, argv, "ptp");
  sleep(1); ros::AsyncSpinner spinner(10); spinner.start();

  std::string tip_link;
  ros::param::param<std::string>("tip_link", tip_link, "hand_base_link");
  PtpAction ptp("cart_ptp", "joint_ptp", tip_link);

  while(ros::ok()){
      sleep(1);
  }

  getchar();

  return 0;
}
