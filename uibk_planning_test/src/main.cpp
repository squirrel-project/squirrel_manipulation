#include <vector>
#include <memory>
#include <iostream>
#include <kukadu/kukadu.hpp>
#include <cmath>

using namespace std;
using namespace arma;
using namespace kukadu;

int main(int argc, char** args) {

    ros::init(argc, args, "uibk_planning_test"); ros::NodeHandle node; sleep(1);
    ros::AsyncSpinner spinner(10); spinner.start();

    cout << "setting up control queue" << endl;
    auto robotinoQueue = KUKADU_SHARED_PTR<KukieControlQueue>(new KukieControlQueue("real", "robotino", node));
    
    cout << "creating moveit kinematics instance" << endl;
    vector<string> controlledJoints{"base_jointx", "base_jointy", "base_jointz", "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"};
    auto mvKin = make_shared<MoveItKinematics>(robotinoQueue, node, "robotino", controlledJoints, "arm_link5");
    
    robotinoQueue->setKinematics(mvKin);
    robotinoQueue->setPathPlanner(mvKin);

    cout << "starting queue" << endl;
    auto realLqThread = robotinoQueue->startQueue();

    cout << "switching to impedance mode if it is not there yet" << endl;
    if(robotinoQueue->getCurrentMode() != KukieControlQueue::KUKA_JNT_POS_MODE) {
        robotinoQueue->stopCurrentMode();
        robotinoQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);
    }

    cout << "deg of freedom: " << robotinoQueue->getMovementDegreesOfFreedom() << endl;
    cout << "joint names size: " << robotinoQueue->getJointNames().size() << endl;

    getchar();

    auto pose = mvKin->computeFk({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    cout << "pose (position) in 0 joint position (x, y, z): " << pose.position.x << " " << pose.position.y << " " << pose.position.z << endl;
    cout << "pose (orientation) in 0 joint position (x, y, z, w): " << pose.orientation.x << " " << pose.orientation.y << " " << pose.orientation.z << " " << pose.orientation.w << endl;
    auto start = stdToArmadilloVec({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    //auto end = stdToArmadilloVec({0.0, 0.0, 0.0, -0.5, 0.0, 0.0, 0.0, -0.1});
    auto end = stdToArmadilloVec({0.0, 0.0, 0.0, -0.5, 0.8, 1.0, 0.0, 0.0});
    
    auto startJacobian = mvKin->getJacobian(armadilloToStdVec(start));
    auto endJacobian = mvKin->getJacobian(armadilloToStdVec(end));
    
    //auto inversePose = mvKin->computeIk(pose).front();
    //cout << "inverse kinematics of pose 0: " << inversePose << endl;
    
    cout << "compute jacobians:" << endl;
    cout << startJacobian << end << endl << endJacobian << endl;

    robotinoQueue->jointPtp(start);
    auto firstJoints = robotinoQueue->getCurrentJoints().joints;
    cout << "current start state: " << firstJoints.t() << endl;
    
    cout << "planning in joint space" << endl;
    auto jointPlan = mvKin->planJointTrajectory({firstJoints, end});
    if(jointPlan.size() > 5) {
		cout << "from: " << jointPlan.front().t() << "to: " << jointPlan.back().t() << endl;
        cout << "joint planning worked (path of length " << jointPlan.size() << " generated) (press key to execute it)" << endl;
        getchar();
        robotinoQueue->setNextTrajectory(jointPlan);
        robotinoQueue->synchronizeToQueue(1);
    }
    
    /*

    cout << "planning in joint space" << endl;
    jointPlan = mvKin->planJointTrajectory({start, end});
    cout << "done with planning" << endl;
    if(jointPlan.size() > 5) {
		cout << "there is something to execute" << endl;
		
		cout << "from: " << jointPlan.front().t() << "to: " << jointPlan.back().t() << endl;
        cout << "joint planning worked (path of length " << jointPlan.size() << " generated) (press key to execute it)" << endl;
        getchar();
        robotinoQueue->setNextTrajectory(jointPlan);
        robotinoQueue->synchronizeToQueue(1);
    }
    
    cout << "planning in joint space" << endl;
    jointPlan = mvKin->planJointTrajectory({robotinoQueue->getCurrentJoints().joints, start});
    if(jointPlan.size() > 5) {
		cout << "from: " << jointPlan.front().t() << "to: " << jointPlan.back().t() << endl;
        cout << "joint planning worked (path of length " << jointPlan.size() << " generated) (press key to execute it)" << endl;
        getchar();
        robotinoQueue->setNextTrajectory(jointPlan);
        robotinoQueue->synchronizeToQueue(1);
    }
    
    */
    
    robotinoQueue->jointPtp(start);
    
    cout << "planning in cartesian space" << endl;
    auto targetPose = mvKin->computeFk({0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2});
    auto targetIk = mvKin->computeIk(armadilloToStdVec(end), targetPose);
    targetPose = mvKin->computeFk({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}); cout << targetPose << endl;
    targetIk = mvKin->computeIk(armadilloToStdVec(end), targetPose); if(targetIk.size() > 0) cout << targetIk.front() << endl; else cout << "no ik solution found" << endl;
    targetPose = mvKin->computeFk({-0.2, 0.2, -0.2, 0.2, -0.2, 0.2, -0.2, 0.2}); cout << targetPose << endl;
    targetIk = mvKin->computeIk(armadilloToStdVec(end), targetPose); if(targetIk.size() > 0) cout << targetIk.front() << endl; else cout << "no ik solution found" << endl;
    targetPose = mvKin->computeFk({0.5, 0.2, 1.0, 0.2, -0.8, 0.2, 0.5, 0.2}); cout << targetPose << endl;
    targetIk = mvKin->computeIk(armadilloToStdVec(end), targetPose); if(targetIk.size() > 0) cout << targetIk.front() << endl; else cout << "no ik solution found" << endl;
    //targetPose = mvKin->computeFk({0.5, 0.2, -1.0, -0.2, -0.8, 0.2, 0.5, -0.2}); cout << targetPose << endl;
    //targetIk = mvKin->computeIk(armadilloToStdVec(end), targetPose); if(targetIk.size() > 0) cout << targetIk.front() << endl; else cout << "no ik solution found" << endl;
    //auto targetPose = mvKin->computeFk({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    //auto targetPose = mvKin->computeFk(firstJoints);
    
    /*
    targetPose.position.x = targetPose.position.y = 0.0;
    targetPose.position.z = 0.8;
    targetPose.orientation.x = 0.0;
    targetPose.orientation.y = 0.0;
    targetPose.orientation.z = -0.5;
    targetPose.orientation.w = 0.9;
    */
    
    cout << "computing ik: " << endl;
    targetIk = mvKin->computeIk(armadilloToStdVec(end), targetPose);
    if(targetIk.size() > 0)
		cout << "target ik: " << targetIk.front() << endl;
	else
		cout << "no ik solution found" << endl;
    jointPlan = mvKin->planCartesianTrajectory(jointPlan.back(), {targetPose});
    auto computedTargetPose = mvKin->computeFk(armadilloToStdVec(jointPlan.back()));
    
    cout << "target pose" << endl;
    cout << "pose (position) in 0 joint position (x, y, z): " << targetPose.position.x << " " << targetPose.position.y << " " << targetPose.position.z << endl;
    cout << "pose (orientation) in 0 joint position (x, y, z, w): " << targetPose.orientation.x << " " << targetPose.orientation.y << " " << targetPose.orientation.z << " " << targetPose.orientation.w << endl;
    
    cout << "computed target pose" << endl;
    cout << "pose (position) in 0 joint position (x, y, z): " << computedTargetPose.position.x << " " << computedTargetPose.position.y << " " << computedTargetPose.position.z << endl;
    cout << "pose (orientation) in 0 joint position (x, y, z, w): " << computedTargetPose.orientation.x << " " << computedTargetPose.orientation.y << " " << computedTargetPose.orientation.z << " " << computedTargetPose.orientation.w << endl;
    
    if(jointPlan.size() > 5) {
		cout << "from: " << jointPlan.front().t() << "to: " << jointPlan.back().t() << endl;
        cout << "cartesian planning worked (path of length " << jointPlan.size() << " generated) (press key to execute it)" << endl;
        getchar();
        robotinoQueue->setNextTrajectory(jointPlan);
        robotinoQueue->synchronizeToQueue(1);
    }
    
    /* for moving with the queue!!!! */
    //vector<double> nextJointPositions = {.....}
    //robotinoQueue->submitNextJointMove(stdToArmadilloVec(nextJointPositions));
    
    /*

    cout << "planning in cartesian space" << endl;
    auto targetPose = mvKin->computeFk({-1.0, -0.5, 0.5, 1.0, -0.5, 0.0});
    jointPlan = mvKin->planCartesianTrajectory(jointPlan.back(), {targetPose});
    if(jointPlan.size() > 5) {
		cout << "from: " << jointPlan.front().t() << "to: " << jointPlan.back().t() << endl;
        cout << "cartesian planning worked (path of length " << jointPlan.size() << " generated) (press key to execute it)" << endl;
        getchar();
        robotinoQueue->setNextTrajectory(jointPlan);
        robotinoQueue->synchronizeToQueue(1);
    }

    auto resultingPose = mvKin->computeFk(armadilloToStdVec(jointPlan.back()));

    cout << "was shooting for " << targetPose.position.x << " " << targetPose.position.y << " " << targetPose.position.z << endl;
    cout << "found a solution for " << resultingPose.position.x << " " << resultingPose.position.y << " " << resultingPose.position.z << endl;

	*/

    getchar();

    return 0;

}
