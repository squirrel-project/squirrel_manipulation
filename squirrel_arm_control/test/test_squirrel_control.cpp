#include "../include/squirrel_arm_control/SquirrelControlQueue.hpp"

#include <iostream>
#include <kukadu/kukadu.h>
#include <boost/program_options.hpp>

using namespace std;
using namespace arma;
using namespace kukadu;

namespace po = boost::program_options;

int main(int argc, char** args) {

    ros::init(argc, args, "test_squirrel_control"); KUKADU_SHARED_PTR<ros::NodeHandle> node = KUKADU_SHARED_PTR<ros::NodeHandle>(new ros::NodeHandle()); usleep(1e6);

    KUKADU_SHARED_PTR<ControlQueue> queue = KUKADU_SHARED_PTR<ControlQueue>(new SquirrelControlQueue(0.1, "Arm", node));

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

    KUKADU_SHARED_PTR<JointDMPLearner> learnerFinalPush = KUKADU_SHARED_PTR<JointDMPLearner>(new JointDMPLearner(az, bz, join_rows(timesFinalPush, dataFinalPush->getJointPos())));

    cout << "training dmp" << endl;
    KUKADU_SHARED_PTR<Dmp> dmpFinalPush = learnerFinalPush->fitTrajectories();

    cout << "creating executor" << endl;
    DMPExecutor execFinalPush(dmpFinalPush, queue);

    cout << "executing dmp" << endl;
    execFinalPush.executeTrajectory(ac, 0, dmpFinalPush->getTmax(), tolAbsErr, tolRelErr);

    cout << "execution done" << endl;

    queue->stopCurrentMode();
    queue->setFinish();
    queueThr->join();

    getchar();

}
