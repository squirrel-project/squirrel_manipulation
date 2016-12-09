#include <vector>
#include <memory>
#include <iostream>
#include <cmath>
#include <boost/thread.hpp>

#include <geometry_msgs/Wrench.h>

#include <kukadu/kukadu.hpp>
#include <squirrel_manipulation_msgs/SoftHandGrasp.h>

#define SENSOR_TOPIC "/wrist"
#define HAND_SERVICE "/softhand_grasp"

using namespace std;
using namespace arma;
using namespace kukadu;

boost::mutex sensor_mutex_;
geometry_msgs::Wrench wrist_sensor_;
bool write_file_set_, writing_;
int stage;
int end_task;

string path_;
string experiment_ = "../../../data/";

void sensorReadCallback(std_msgs::Float64MultiArray msg);

void dataStore();

int main(int argc, char** args) {

    ros::init(argc, args, "handover_data_collection");
    ros::NodeHandle node;
    sleep(1);
    boost::thread* data_store_ = new boost::thread(boost::bind(dataStore));


    ros::AsyncSpinner spinner(10); spinner.start();
    //wrist sensor
    node.subscribe(SENSOR_TOPIC, 1, &sensorReadCallback);

    //hand
    squirrel_manipulation_msgs::SoftHandGrasp graspService;
    graspService.request.position = 1.0;
    squirrel_manipulation_msgs::SoftHandGrasp releaseService;
    releaseService.request.position = 0.0;


    //arm control

    ROS_INFO("(handover) setting up control queue");
    cout <<  endl;
    auto robotinoQueue = KUKADU_SHARED_PTR<KukieControlQueue>(new KukieControlQueue("real", "robotino", node));

    ROS_INFO("(handover) creating moveit kinematics instance");
    cout << endl;
    vector<string> controlledJoints{"base_jointx", "base_jointy", "base_jointz", "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"};
    auto mvKin = make_shared<MoveItKinematics>(robotinoQueue, node, "robotino", controlledJoints, "arm_link5");

    robotinoQueue->setKinematics(mvKin);
    robotinoQueue->setPathPlanner(mvKin);

    ROS_INFO("(handover) starting queue");
    cout << endl;
    auto realLqThread = robotinoQueue->startQueue();

    cout << "switching to impedance mode if it is not there yet" << endl;
    if(robotinoQueue->getCurrentMode() != KukieControlQueue::KUKA_JNT_POS_MODE) {
        robotinoQueue->stopCurrentMode();
        robotinoQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);
    }

    auto start = stdToArmadilloVec({0.0, 0.0, 0.0, 1.0, -0.5, 1.4, 1.0, 1.6});
    auto end = stdToArmadilloVec({0.0, 0.0, 0.0, 0.7, 0.7, 1.4, 1.0, 1.6});

    int grasp_value = 100;
    stage = 100;
     end_task = 100;

    write_file_set_ = false;
    while(end_task > 0) {

        cout << "(handover) enter experiment name: ";
        cin >> experiment_;

        writing_ = true;
        auto firstJoints = robotinoQueue->getCurrentJoints().joints;
        cout << "(handover) current robot state: " << firstJoints.t() << endl;

        ROS_INFO("(handover) going to initial pose with the open hand");
        stage = 0;
        robotinoQueue->jointPtp(start);


        stage = 1; // initial pose with the open hand
        firstJoints = robotinoQueue->getCurrentJoints().joints;
        cout << "(handover) current robot state: " << firstJoints.t() << endl;

        ROS_INFO("(handover) going to the handover pose with the open hand");
        stage = 2;
        robotinoQueue->jointPtp(end);

        firstJoints = robotinoQueue->getCurrentJoints().joints;
        cout << "(handover) current robot state: " << firstJoints.t() << endl;

        ROS_INFO("(handover) waiting to grasp the object");
        stage = 3;

        cout << "(handover) press 1 to close the hand" << endl;
        cin >> grasp_value;
        if(grasp_value == 1){
            stage = 4; //grasping the object
            cout<<"OK"<<endl;

            if ( ros::service::call(HAND_SERVICE, graspService) ){
                ROS_INFO("HAND Grasped!");
            }else{
                ROS_ERROR("FAILED to Graps!");
            }
        }

        ROS_INFO("(handover) going to initial pose with the closed hand");
        stage = 5;
        robotinoQueue->jointPtp(start);

        stage = 6; // initial pose with the closed hand
        firstJoints = robotinoQueue->getCurrentJoints().joints;
        cout << "(handover) current robot state: " << firstJoints.t() << endl;

        ROS_INFO("(handover) going to the handover pose with the closed hand");
        stage = 7;
        robotinoQueue->jointPtp(end);


        ROS_INFO("(handover) waiting to release the object");
        stage = 8;

        cout << "(handover) press 1 to open the hand" << endl;
        cin >> grasp_value;
        if(grasp_value == 1){
            stage = 9   ; //releasing the object
            cout<<"OK"<<endl;
            if ( ros::service::call(HAND_SERVICE, releaseService) ){
                ROS_INFO("HAND Released!");
            }else{
                ROS_ERROR("FAILED to Release!");
            }
        }

        ROS_INFO("(handover) going to initial pose with the open hand");
        stage = 0;
        robotinoQueue->jointPtp(start);

        cout << "(handover) press 1 to start handover / 0 to exit" << endl;
        cin >> end_task;
        writing_ = false;


    }
    data_store_->join();

    getchar();

    return 0;

}

void sensorReadCallback(std_msgs::Float64MultiArray msg){

    wrist_sensor_.force.x=msg.data.at(0);
    wrist_sensor_.force.y=msg.data.at(1);
    wrist_sensor_.force.z=msg.data.at(2);

    wrist_sensor_.torque.x=msg.data.at(3);
    wrist_sensor_.torque.y=msg.data.at(4);
    wrist_sensor_.torque.z=msg.data.at(5);
}

void dataStore(){
    ros::Rate lRate(20.0);
    bool closed = false;


    while(end_task>0&&!closed){
        closed = true;

        std::ofstream rFile;

        string nameF = path_  + experiment_ + ".txt";

        if(!write_file_set_ && writing_){
            cout <<"open file "<<endl;
            write_file_set_ = true;
            rFile.open(nameF.c_str());
            rFile<< "time" << "\t" << "stage" << "\t" <<"force.x" << "\t" <<"force.y" << "\t" <<"force.z" << "\t" <<"torque.x" << "\t" <<"torque.y" <<"\t" <<"torque.z" << endl;
        }
        if(write_file_set_&&writing_){

            rFile << ros::Time::now().toSec() << "\t";
            rFile << stage <<"\t";

            rFile << wrist_sensor_.force.x << "\t";
            rFile << wrist_sensor_.force.y << "\t";
            rFile << wrist_sensor_.force.z << "\t";
            rFile << wrist_sensor_.torque.x << "\t";
            rFile << wrist_sensor_.torque.y << "\t";
            rFile << wrist_sensor_.torque.z << "\t";

            rFile << endl;

        }

        if(write_file_set_ && !writing_){
            cout<<"closing file"<<endl;
            write_file_set_ = false;
            rFile.close();
            closed = false;
        }
        lRate.sleep();
    }

}
