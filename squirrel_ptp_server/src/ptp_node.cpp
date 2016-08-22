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

	bool ptpRunning;
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

public:

	PtpAction(std::string name, std::string jointPtpName) :
		as_(nh_, name, boost::bind(&PtpAction::executeCB, this, _1), false),
		jointPtpAs_(nh_, jointPtpName, boost::bind(&PtpAction::executeJointPtp, this, _1), false),
		action_name_(name) {
			
		this->jointPtpName = jointPtpName;
		
		ptpRunning = false;
			
		cout << "setting up control queue" << endl;
		robotinoQueue = KUKADU_SHARED_PTR<KukieControlQueue>(new KukieControlQueue("real", "robotino", nh_));
		
		cout << "creating moveit kinematics instance" << endl;
		vector<string> controlledJoints{"base_jointx", "base_jointy", "base_jointz", "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"};
		mvKin = make_shared<MoveItKinematics>(robotinoQueue, nh_, "robotino", controlledJoints, "arm_link5");
		
		robotinoQueue->setKinematics(mvKin);
		robotinoQueue->setPathPlanner(mvKin);

		cout << "starting queue" << endl;
		auto realLqThread = robotinoQueue->startQueue();
		
		as_.start();
		jointPtpAs_.start();
		
	}

	~PtpAction(void) {
	}
	
	void executeJointPtp(const squirrel_manipulation_msgs::JointPtpGoalConstPtr& goal) {
		
		ptpRunning = true;
		
		if(!ptpRunning) {
		
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
		
		ptpRunning = false;
		
	}

	void executeCB(const squirrel_manipulation_msgs::PtpGoalConstPtr &goal) {
		
		ptpRunning = true;
		
		if(!ptpRunning) {
		
			if(robotinoQueue->getCurrentMode() != KukieControlQueue::KUKA_JNT_POS_MODE) {
				robotinoQueue->stopCurrentMode();
				robotinoQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);
			}
			
			feedback_.current_status = "start planning";
			as_.publishFeedback(feedback_);
			auto jointPlan = mvKin->planCartesianTrajectory({goal->pose});
			
			if(jointPlan.size() > 5) {
				feedback_.current_status = "start execution";
				as_.publishFeedback(feedback_);
				robotinoQueue->setNextTrajectory(jointPlan);
				robotinoQueue->synchronizeToQueue(1);
			}
			
			robotinoQueue->stopCurrentMode();
			
			result_.result_status = "execution done";
			as_.setSucceeded(result_);
			
		} else {
			
			jptpresult_.result_status = "execution failed because another ptp is currently running";
			jointPtpAs_.setSucceeded(jptpresult_);
			
		}
		
		ptpRunning = false;

	}

};


int main(int argc, char** argv) {

	ros::init(argc, argv, "ptp");
	sleep(1); ros::AsyncSpinner spinner(10); spinner.start();

	PtpAction ptp("cart_ptp", "joint_ptp");
	
	getchar();

	return 0;
}
