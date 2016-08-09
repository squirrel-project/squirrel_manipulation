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

    auto pose = mvKin->computeFk({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    cout << "pose in 0 joint position: " << pose.position.x << " " << pose.position.y << " " << pose.position.z << endl;

    auto start = stdToArmadilloVec({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    auto end = stdToArmadilloVec({0.0, 0.5, 0.0, -0.5, 0.0, 0.0, 0.0, -0.1});
    
    auto startJacobian = mvKin->getJacobian(armadilloToStdVec(start));
    auto endJacobian = mvKin->getJacobian(armadilloToStdVec(end));
    
    //auto inversePose = mvKin->computeIk(pose).front();
    //cout << "inverse kinematics of pose 0: " << inversePose << endl;
    
    cout << "compute jacobians:" << endl;
    cout << startJacobian << end << endl << endJacobian << endl;

    //robotinoQueue->jointPtp(start);
    /*
    auto firstJoints = robotinoQueue->getCurrentJoints().joints;
    cout << "current start state: " << firstJoints.t() << endl;
    
    cout << "planning in joint space" << endl;
    auto jointPlan = mvKin->planJointTrajectory({firstJoints, start});
    if(jointPlan.size() > 5) {
		cout << "from: " << jointPlan.front().t() << "to: " << jointPlan.back().t() << endl;
        cout << "joint planning worked (path of length " << jointPlan.size() << " generated) (press key to execute it)" << endl;
        getchar();
        robotinoQueue->setNextTrajectory(jointPlan);
        robotinoQueue->synchronizeToControlQueue(1);
    }
    */

    cout << "planning in joint space" << endl;
    auto jointPlan = mvKin->planJointTrajectory({start, end});
    cout << "done with planning" << endl;
    if(jointPlan.size() > 5) {
		cout << "there is something to execute" << endl;
		
		cout << "from: " << jointPlan.front().t() << "to: " << jointPlan.back().t() << endl;
        cout << "joint planning worked (path of length " << jointPlan.size() << " generated) (press key to execute it)" << endl;
        getchar();
        robotinoQueue->setNextTrajectory(jointPlan);
        robotinoQueue->synchronizeToQueue(1);
    }
    
    /* for moving with the queue!!!! */
    //vector<double> nextJointPositions = {.....}
    //robotinoQueue->submitNextJointMove(stdToArmadilloVec(nextJointPositions));
    
    /*
    
    cout << "planning in joint space" << endl;
    jointPlan = mvKin->planJointTrajectory({robotinoQueue->getCurrentJoints().joints, start});
    if(jointPlan.size() > 5) {
		cout << "from: " << jointPlan.front().t() << "to: " << jointPlan.back().t() << endl;
        cout << "joint planning worked (path of length " << jointPlan.size() << " generated) (press key to execute it)" << endl;
        getchar();
        robotinoQueue->setNextTrajectory(jointPlan);
        robotinoQueue->synchronizeToQueue(1);
    }

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
