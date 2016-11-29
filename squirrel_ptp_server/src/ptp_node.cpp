#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <squirrel_manipulation_msgs/PtpAction.h>
#include <squirrel_manipulation_msgs/JointPtpAction.h>

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
  actionlib::SimpleActionServer<squirrel_manipulation_msgs::PtpAction> as_;
  actionlib::SimpleActionServer<squirrel_manipulation_msgs::JointPtpAction> jointPtpAs_;
  std::string action_name_;
  std::string jointPtpName;
  // create messages that are used to published feedback/result
  squirrel_manipulation_msgs::PtpFeedback feedback_;
  squirrel_manipulation_msgs::PtpResult result_;
  
  squirrel_manipulation_msgs::JointPtpFeedback jptpfeedback_;
  squirrel_manipulation_msgs::JointPtpResult jptpresult_;
  
  KUKADU_SHARED_PTR<KukieControlQueue> robotinoQueue;
  KUKADU_SHARED_PTR<MoveItKinematics> mvKin;
  KUKADU_SHARED_PTR<kukadu_thread> realLqThread;

public:

  PtpAction(std::string name, std::string jointPtpName, std::string tip_link) :
    as_(nh_, name, boost::bind(&PtpAction::executeCB, this, _1), false),
    jointPtpAs_(nh_, jointPtpName, boost::bind(&PtpAction::executeJointPtp, this, _1), false),
    action_name_(name) {
      
    this->jointPtpName = jointPtpName;
    
    cartPtpRunning = false;
    jointPtpRunning = false;
      
    cout << "setting up control queue" << endl;
    robotinoQueue = KUKADU_SHARED_PTR<KukieControlQueue>(new KukieControlQueue("real", "robotino", nh_));
    
    cout << "creating moveit kinematics instance" << endl;
    vector<string> controlledJoints{"base_jointx", "base_jointy", "base_jointz", "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"};
    //mvKin = make_shared<MoveItKinematics>(robotinoQueue, nh_, "robotino", controlledJoints, "arm_link5", false, 10, 2.0);
    mvKin = make_shared<MoveItKinematics>(robotinoQueue, nh_, "robotino", controlledJoints, "hand_palm_link", false, 10, 2.0);
    
    robotinoQueue->setKinematics(mvKin);
    robotinoQueue->setPathPlanner(mvKin);

    cout << "starting queue" << endl;           
    realLqThread = robotinoQueue->startQueue();
    
    as_.start();
    jointPtpAs_.start();
    
  }

  ~PtpAction(void) {
  }
  
  void executeJointPtp(const squirrel_manipulation_msgs::JointPtpGoalConstPtr& goal) {
    
    jointPtpRunning = true;
    
    if(!cartPtpRunning) {
        
      if(robotinoQueue->getCurrentMode() != KukieControlQueue::KUKA_JNT_POS_MODE) {
        robotinoQueue->stopCurrentMode();
        robotinoQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);
      }
      
      jptpfeedback_.current_status = "start planning";
      jointPtpAs_.publishFeedback(jptpfeedback_);
      robotinoQueue->jointPtp(goal->joints.data);
      
      robotinoQueue->stopCurrentMode();
      
      jptpresult_.result_status = "execution done";
      jointPtpAs_.setSucceeded(jptpresult_);
      
    } else {
      
      jptpresult_.result_status = "execution failed because another ptp is currently running";
      jointPtpAs_.setSucceeded(jptpresult_);
      
    }
    
    jointPtpRunning = false;
    
  }

  void executeCB(const squirrel_manipulation_msgs::PtpGoalConstPtr &goal) {

    cartPtpRunning = true;

    if(!jointPtpRunning) {
      if(robotinoQueue->getCurrentMode() != KukieControlQueue::KUKA_JNT_POS_MODE) {
        robotinoQueue->stopCurrentMode();
        robotinoQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);
      }
      feedback_.current_status = "executing";
      as_.publishFeedback(feedback_);

      bool succeded=false;
      try{
        robotinoQueue->cartesianPtp(goal->pose);
        succeded=true;
      } catch (KukaduException& ex) {
        cout << "No Ik Found: retry." << endl;
      }
      robotinoQueue->stopCurrentMode();
      if (succeded) {
        result_.result_status = "execution done.";
      } else {
        result_.result_status = "execution failed.";                
      }
      as_.setSucceeded(result_);
      
    } else {
      jptpresult_.result_status = "execution failed because another ptp is currently running";
      jointPtpAs_.setSucceeded(jptpresult_);      
    }
    
    cartPtpRunning = false;

  }

};


int main(int argc, char** argv) {

  ros::init(argc, argv, "ptp");
  sleep(1); ros::AsyncSpinner spinner(10); spinner.start();

  std::string tip_link;
  ros::param::param<std::string>("tip_link", tip_link, "hand_base_link");
  PtpAction ptp("cart_ptp", "joint_ptp", tip_link);
  
  getchar();

  return 0;
}
