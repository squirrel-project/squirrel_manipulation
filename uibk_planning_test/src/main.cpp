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

    auto pose = mvKin->computeFk({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    cout << "pose (position) in 0 joint position (x, y, z): " << pose.position.x << " " << pose.position.y << " " << pose.position.z << endl;
    cout << "pose (orientation) in 0 joint position (x, y, z, w): " << pose.orientation.x << " " << pose.orientation.y << " " << pose.orientation.z << " " << pose.orientation.w << endl;
    auto start = stdToArmadilloVec({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    //auto end = stdToArmadilloVec({0.0, 0.0, 0.0, -0.5, 0.0, 0.0, 0.0, -0.1});
    auto end = stdToArmadilloVec({0.0, 0.0, 0.0, -0.5, 0.8, 1.0, 0.0, 0.0});
    
    auto startJacobian = mvKin->getJacobian(armadilloToStdVec(start));
    auto endJacobian = mvKin->getJacobian(armadilloToStdVec(end));
    
    cout << "compute jacobians:" << endl;
    cout << startJacobian << end << endl << endJacobian << endl;

    //robotinoQueue->jointPtp(start);
    auto firstJoints = robotinoQueue->getCurrentJoints().joints;
    cout << "current start state: " << firstJoints.t() << endl;

    auto targetPose = mvKin->computeFk({0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2});
    auto targetIk = mvKin->computeIk(armadilloToStdVec(end), targetPose);
    targetPose = mvKin->computeFk({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}); cout << targetPose << endl;
    targetIk = mvKin->computeIk(armadilloToStdVec(end), targetPose); if(targetIk.size() > 0) cout << targetIk.front() << endl; else cout << "no ik solution found" << endl;
    targetPose = mvKin->computeFk({-0.2, 0.2, -0.2, 0.2, -0.2, 0.2, -0.2, 0.2}); cout << targetPose << endl;
    targetIk = mvKin->computeIk(armadilloToStdVec(end), targetPose); if(targetIk.size() > 0) cout << targetIk.front() << endl; else cout << "no ik solution found" << endl;
    targetPose = mvKin->computeFk({0.5, 0.2, 1.0, 0.2, -0.8, 0.2, 0.5, 0.2}); cout << targetPose << endl;
    targetIk = mvKin->computeIk(armadilloToStdVec(end), targetPose); if(targetIk.size() > 0) cout << targetIk.front() << endl; else cout << "no ik solution found" << endl;

    int choice = 100;
    while(choice > 0) {

        cout << endl << endl << endl << "what do you want to do?" << endl << endl;
        cout << "(1) joint ptp to [0,0,0,0,0,0,0,0]" << endl;
        cout << "(2) joint ptp to [0.0, 0.0, 0.0, -0.5, 0.8, 1.0, 0.0, 0.0]" << endl;
        cout << "(3) joint ptp to [x,x,x,x,x,x,x,x]" << endl;
        cout << "(4) cartesian ptp to [" << pose.position.x << ", " << pose.position.y << ", " << pose.position.z <<
                "] [" << pose.orientation.x << ", " << pose.orientation.y << ", " << pose.orientation.z << ", " << pose.orientation.w << "]" << endl;
        cout << "(5) cartesian ptp to [x,x,x] [" << pose.orientation.x << ", " << pose.orientation.y << ", " << pose.orientation.z << ", " << pose.orientation.w << "]" << endl;
        cout << "(6) sin swing in joint 4" << endl;
        cin >> choice;
        fflush(stdin);

        arma::vec desiredJoints(8);
        auto tmpPose = pose;

        switch(choice) {
        case 1:
            robotinoQueue->jointPtp(start);
            break;
        case 2:
            robotinoQueue->jointPtp(end);
            break;
        case 3:
            cout << "give the desired joint position [x,x,x,x,x,x,x,x]" << endl;
            cin >> desiredJoints(0) >> desiredJoints(1) >> desiredJoints(2) >> desiredJoints(3) >>
                   desiredJoints(4) >> desiredJoints(5) >> desiredJoints(6) >> desiredJoints(7);
            fflush(stdin);
            robotinoQueue->jointPtp(desiredJoints);
            break;
        case 4:
            robotinoQueue->cartesianPtp(pose);
            break;
        case 5:
            pose = robotinoQueue->getCurrentCartesianPose();
            cout << "give the desired end-effector position (no rotation required) (current position [" << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << "])" << endl;
            cin >> tmpPose.position.x >> tmpPose.position.y >> tmpPose.position.z;
            fflush(stdin);
            robotinoQueue->cartesianPtp(tmpPose);
            break;
        case 6:
            double stepsPerSecond = 200.0;
            robotinoQueue->jointPtp(start);
            auto centerJoints = robotinoQueue->getCurrentJoints().joints;
            for(double i = 0.0; true; i += 0.1 / stepsPerSecond) {
                auto submitJoints = centerJoints;
                submitJoints(3) += sin(i);
                robotinoQueue->move(submitJoints);
                robotinoQueue->synchronizeToQueue(1);
            }
        }

    }

    getchar();

    return 0;

}
