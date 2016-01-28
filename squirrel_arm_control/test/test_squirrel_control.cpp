#include "../include/squirrel_arm_control/SquirrelControlQueue.hpp"

#include <iostream>
#include <kukadu/kukadu.h>
#include <boost/program_options.hpp>

#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

using namespace std;
using namespace arma;
using namespace kukadu;

namespace po = boost::program_options;

int main(int argc, char** args) {

    ros::init(argc, args, "test_squirrel_control"); KUKADU_SHARED_PTR<ros::NodeHandle> node = KUKADU_SHARED_PTR<ros::NodeHandle>(new ros::NodeHandle()); usleep(1e6);

    KUKADU_SHARED_PTR<ControlQueue> queue = KUKADU_SHARED_PTR<ControlQueue>(new SquirrelControlQueue(0.1, "arm", node));

    ros::AsyncSpinner spinner(5);
    spinner.start();


    /* topics are named different in sim and real
     *
     * in sim its called /arm_controller/joint_states and in real its /robotino_arm/arm/joint_states
     *
     * */


































    double tau, az, bz, dmpStepSize, tolAbsErr, tolRelErr, ac;

    po::options_description desc("Allowed options");
    desc.add_options()
            ("dmp.tau", po::value<double>(), "tau")
            ("dmp.az", po::value<double>(), "az")
            ("dmp.bz", po::value<double>(), "bz")
            ("dmp.dmpStepSize", po::value<double>(), "dmp time step size")
            ("dmp.tolAbsErr", po::value<double>(), "tolerated absolute error")
            ("dmp.tolRelErr", po::value<double>(), "tolerated relative error")
            ("dmp.ac", po::value<double>(), "ac")
    ;

    ifstream parseFile(resolvePath("$KUKADU_HOME/cfg/guided.prop").c_str(), std::ifstream::in);
    po::variables_map vm;
    po::store(po::parse_config_file(parseFile, desc), vm);
    po::notify(vm);

    if (vm.count("dmp.tau")) tau = vm["dmp.tau"].as<double>();
    else return 1;
    if (vm.count("dmp.az")) az = vm["dmp.az"].as<double>();
    else return 1;
    if (vm.count("dmp.bz")) bz = vm["dmp.bz"].as<double>();
    else return 1;
    if (vm.count("dmp.dmpStepSize")) dmpStepSize = vm["dmp.dmpStepSize"].as<double>();
    else return 1;
    if (vm.count("dmp.tolAbsErr")) tolAbsErr = vm["dmp.tolAbsErr"].as<double>();
    else return 1;
    if (vm.count("dmp.tolRelErr")) tolRelErr = vm["dmp.tolRelErr"].as<double>();
    else return 1;
    if (vm.count("dmp.ac")) ac = vm["dmp.ac"].as<double>();
    else return 1;

    cout << "all properties loaded" << endl;
    string storeDir = resolvePath("/tmp/robotino_teaching_demo_guided");
    deleteDirectory(storeDir);

    KUKADU_SHARED_PTR<kukadu_thread> queueThr = queue->startQueueThread();
    queue->switchMode(ControlQueue::CONTROLQUEUE_JNT_POS_MODE);

    /*
    queue->jointPtp(stdToArmadilloVec(createJointsVector(8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)));
    getchar(); cout << "current cartesian pos: " << queue->getCurrentCartesianPose() << endl;
    */

    geometry_msgs::Pose origin;
    origin.position.x = origin.position.y = 0.0;
    origin.position.z = 0.85;
    origin.orientation.x = 0.00245218;
    origin.orientation.y = 0.00422713;
    origin.orientation.z = -0.49833;
    origin.orientation.w = 0.866973;

    cout << "ready to perfom cartesian ptp?" << endl;
    getchar();

    queue->cartesianPtp(origin);

    cout << "done" << endl;
    getchar();

    geometry_msgs::Pose newPose;
    newPose.position.x = 0.0867546;
    newPose.position.y = -0.151025;
    newPose.position.z = 0.726772;
    newPose.orientation.x = 0.114398;
    newPose.orientation.y = 0.160558;
    newPose.orientation.z = -0.407022;
    newPose.orientation.w = 0.89189;

    cout << "ready to perfom cartesian ptp?" << endl;
    getchar();

    queue->cartesianPtp(newPose);

    cout << "done" << endl;
    getchar();

    /*
    queue->jointPtp(stdToArmadilloVec(createJointsVector(8, 0.2, 0.2, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0)));
    getchar(); cout << "current cartesian pos: " << queue->getCurrentCartesianPose() << endl;
    queue->jointPtp(stdToArmadilloVec(createJointsVector(8, 0.4, 0.2, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0)));
    getchar(); cout << "current cartesian pos: " << queue->getCurrentCartesianPose() << endl;
    queue->jointPtp(stdToArmadilloVec(createJointsVector(8, 0.6, 0.2, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0)));
    getchar(); cout << "current cartesian pos: " << queue->getCurrentCartesianPose() << endl;
    queue->jointPtp(stdToArmadilloVec(createJointsVector(8, 0.8, 0.2, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0)));
    getchar(); cout << "current cartesian pos: " << queue->getCurrentCartesianPose() << endl;
    queue->jointPtp(stdToArmadilloVec(createJointsVector(8, 1.0, 0.2, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0)));
    getchar(); cout << "current cartesian pos: " << queue->getCurrentCartesianPose() << endl;
    queue->jointPtp(stdToArmadilloVec(createJointsVector(8, 1.2, 0.2, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0)));
    getchar(); cout << "current cartesian pos: " << queue->getCurrentCartesianPose() << endl;
    queue->jointPtp(stdToArmadilloVec(createJointsVector(8, 1.4, 0.2, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0)));
    cout << "that was the last one" << endl;
    getchar(); cout << "current cartesian pos: " << queue->getCurrentCartesianPose() << endl;
    */

    /*

    vector<KUKADU_SHARED_PTR<ControlQueue> > queueVectors;
    queueVectors.push_back(queue);

    cout << "starting measurement" << endl;
    SensorStorage scaredOfSenka(queueVectors, std::vector<KUKADU_SHARED_PTR<GenericHand> >(), 1000);
    scaredOfSenka.setExportMode(SensorStorage::STORE_TIME | SensorStorage::STORE_RBT_CART_POS | SensorStorage::STORE_RBT_JNT_POS);
    scaredOfSenka.startDataStorage(storeDir);
    cout << "measuerment started" << endl;

    vec firstJoints = queue->getCurrentJoints().joints;
    double firstJoint0 = firstJoints(0);
    for(int i = 0; i < 80; ++i) {
        firstJoints(0) = firstJoint0 + 0.3 * sin(i / 25.0);
        queue->addJointsPosToQueue(firstJoints);
    }
    sleep(20);
    scaredOfSenka.stopDataStorage();

    cout << "measurement finished" << endl;

    cout << "reading data" << endl;
    KUKADU_SHARED_PTR<SensorData> dataFinalPush = SensorStorage::readStorage(queue, storeDir + string("/") + queue->getRobotFileName() + string("_0"));

    cout << "retrieving time values" << endl;
    vec timesFinalPush = dataFinalPush->getTimes();
    */

    /*
    KUKADU_SHARED_PTR<JointDMPLearner> learnerFinalPush = KUKADU_SHARED_PTR<JointDMPLearner>(new JointDMPLearner(az, bz, join_rows(timesFinalPush, dataFinalPush->getJointPos())));

    cout << "training dmp" << endl;
    KUKADU_SHARED_PTR<Dmp> dmpFinalPush = learnerFinalPush->fitTrajectories();

    cout << "creating executor" << endl;
    DMPExecutor execFinalPush(dmpFinalPush, queue);

    cout << "executing dmp" << endl;
    execFinalPush.executeTrajectory(ac, 0, dmpFinalPush->getTmax(), tolAbsErr, tolRelErr);

    cout << "execution done" << endl;
    */

    queue->stopCurrentMode();
    queue->setFinish();
    queueThr->join();









    /*
    int success;
    moveit::planning_interface::MoveGroup group("arm");
    moveit::planning_interface::MoveGroup::Plan my_plan;

    ros::Publisher display_publisher = node->advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    std::vector<double> group_variable_values;
    group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

    for(int i = 0; i < group_variable_values.size(); ++i) {
        cout << group_variable_values.at(i) << " ";
    }
    cout << endl;
    */

    /*

    group_variable_values[3] = 0.2;
    group.setJointValueTarget(group_variable_values);

    cout << "starting plan" << endl;
    success = group.plan(my_plan);
    cout << "finished planning" << endl;

    cout << "success: " << success << endl;

    ROS_INFO("Visualizing plan 1 (again)");
    display_trajectory.trajectory_start = my_plan.start_state_;
    display_trajectory.trajectory.push_back(my_plan.trajectory_);
    display_publisher.publish(display_trajectory);

    cout << "waiting for display" << endl;
    sleep(5);

    cout << "start execution" << endl;

    group.execute(my_plan);

    cout << "execution done" << endl;
    */

    /*
    sleep(5);

    group.execute(myp_plan);
    */

    getchar();

}
