#include <squirrel_handovers/handover.hpp>


using namespace std;
using namespace arma;
using namespace kukadu;

HandoverAction::HandoverAction(const std::string std_HandoverServerActionName) :
    handoverServer(nh, std_HandoverServerActionName, boost::bind(&HandoverAction::executeHandover, this, _1), false),
    runHandover_ (true),
    private_nh("~")
{

    private_nh.param("base_frame", base_frame_, std::string("/base_link"));
    private_nh.param("wrist_frame", wrist_frame_, std::string("/arm_link5"));
    private_nh.param("tuw_robotino", tuw_robotino, std::string("tuw-robotino2"));
    private_nh.param("uibk_robotino", uibk_robotino, std::string("uibk-robotino2-sh"));
    private_nh.param("robot", robot, std::string("uibk-robotino2-sh"));
    private_nh.param("frequency", handover_frequency_, 10.00);

    //set callback for cancel request
    handoverServer.registerPreemptCallback(boost::bind(&HandoverAction::preemptCB, this));  


    vector<double> temp (3,0);
    current_forces_ = temp;
    current_torques_ = temp;

    sub_h = nh.subscribe(SENSOR_TOPIC, 1, &HandoverAction::sensorReadCallbackWrist,this);
    tiltPub = nh.advertise<std_msgs::Float64>(TILT_TOPIC, 1);
    panPub = nh.advertise<std_msgs::Float64>(PAN_TOPIC, 1);
    safety_pub_ = nh.advertise<std_msgs::Bool>(SAFETY_TOPIC,1);


    if(robot == tuw_robotino){
        sub_f = nh.subscribe(FINGERTIP_TOPIC, 1, &HandoverAction::sensorReadCallbackFingers, this);
    }

    handoverServer.start();
    ROS_INFO("(Handover) server started \n");
    cout<<endl;

}

void HandoverAction::executeHandover(const squirrel_manipulation_msgs::HandoverGoalConstPtr &goal) {


    ROS_INFO("(Handover) action started \n");
    //cout <<  endl;

    runHandover_ = true;
    ros::Rate lRate(handover_frequency_);

    ROS_INFO("(Handover) action type %s \n", goal->action_type.c_str());
    ROS_INFO("(Handover) robot: %s \n", robot.c_str());

    //hand
    squirrel_manipulation_msgs::SoftHandGrasp graspService;
    graspService.request.position = 0.9;
    squirrel_manipulation_msgs::SoftHandGrasp releaseService;
    releaseService.request.position = 0.0;

    actionlib::SimpleActionClient<kclhand_control::ActuateHandAction> kclhandGraspActionClient("hand_controller/actuate_hand", true);
    if (robot == tuw_robotino){
        kclhandGraspActionClient.waitForServer();}
    kclhand_control::ActuateHandActionGoal graspServiceKCL;
    graspServiceKCL.goal.command = 1.0;
    graspServiceKCL.goal.force_limit = 1.0;
    kclhand_control::ActuateHandActionGoal releaseServiceKCL;
    releaseServiceKCL.goal.command = 0.0;
    releaseServiceKCL.goal.force_limit = 1.0;

    sleep (0.2);

    //arm control

    ROS_INFO("(handover) setting up control queue \n");
    //cout <<  endl;
    auto robotinoQueue = KUKADU_SHARED_PTR<KukieControlQueue>(new KukieControlQueue("real", "robotino", nh));

    ROS_INFO("(handover) creating moveit kinematics instance");
    //cout << endl;
    vector<string> controlledJoints{"base_jointx", "base_jointy", "base_jointz", "arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"};
    auto mvKin = make_shared<MoveItKinematics>(robotinoQueue, nh, "robotino", controlledJoints, "arm_link5");

    robotinoQueue->setKinematics(mvKin);
    robotinoQueue->setPathPlanner(mvKin);

    ROS_INFO("(handover) starting queue \n");
    //cout << endl;
    auto realLqThread = robotinoQueue->startQueue();

    if(robotinoQueue->getCurrentMode() != KukieControlQueue::KUKA_JNT_POS_MODE) {
        robotinoQueue->stopCurrentMode();
        robotinoQueue->switchMode(KukieControlQueue::KUKA_JNT_POS_MODE);
    }

    auto firstJoints = robotinoQueue->getCurrentJoints().joints;

    auto start = stdToArmadilloVec({0.0, 0.0, 0.0, 1.0, -0.5, 1.4, 1.0, 1.6});
    auto end = stdToArmadilloVec({0.0, 0.0, 0.0, 0.7, 0.7, 1.4, 1.0, 1.6});
    auto handover_vienna = stdToArmadilloVec({0.0, 0.0, 0.0, 1.0, 1.0, 0.0, -1.2, -1.9});
    auto start_vienna = stdToArmadilloVec({0.0, 0.0, 0.0, 0.7, 1.6, 0.0, -1.8, -1.9});
    auto end_type1 = stdToArmadilloVec({0.0, 0.0, 0.0, 0.7, 0.7, 1.4, 1.0, 1.6});
    auto end_type2 = stdToArmadilloVec({0.0, 0.0, 0.0, 0.7, 0.7, 1.4, 1.0, 1.6});
    auto end_type3 = stdToArmadilloVec({0.0, 0.0, 0.0, 0.7, 0.7, 1.4, 1.0, 1.6});
    auto end_type4 = stdToArmadilloVec({0.0, 0.0, 0.0, 0.7, 0.7, 1.4, 1.0, 1.6});

    //robot shall not move base

    copy(firstJoints.begin(), firstJoints.begin() + 3, start.begin());
    copy(firstJoints.begin(), firstJoints.begin() + 3, end.begin());


    //handover type

    if (goal->handover_type == 1){
        copy(end_type1.begin() + 3, end_type1.end(), end.begin() + 3);
    }
    else if (goal->handover_type == 2){
        copy(end_type2.begin() + 3, end_type2.end(), end.begin() + 3);

    }
    else if (goal->handover_type == 3){
        copy(end_type3.begin() + 3, end_type3.end(), end.begin() + 3);

    }
    else if (goal->handover_type == 4){
        copy(end_type4.begin() + 3, end_type4.end(), end.begin() + 3);
    }

    if (robot == tuw_robotino){
        copy(start_vienna.begin() + 3, start_vienna.end(), start.begin() + 3);
        copy(handover_vienna.begin() + 3, handover_vienna.end(), end.begin() + 3);
    }

    bool handover_success_ = false;
    stage = 0;
    
    std::string take ("take");
    std::string give ("give");

    torque_past.clear();
    force_past.clear();

    if(take.compare(goal->action_type.c_str())==0 && runHandover_){
        sleep(0.5);
        int k = 0;
        force_past.clear();
        torque_past.clear();
        while (k < 3){
            record_magnitude_simple(current_forces_, current_torques_);
            lRate.sleep();
            k++;
        }
        double init_magnitude = getMean(force_past);
        this->resetSafety();

        //hand  should be closed
        if (robot == uibk_robotino && runHandover_){

            if (ros::service::call(HAND_SERVICE, graspService) ){
                ROS_INFO("(handover) (hand msg) HAND Closed! \n");
                handover_success_ = true;
            }else{
                ROS_ERROR("handover)(hand msg) FAILED to Closed! \n");
                handover_success_ = false;
                runHandover_ = false;
            }
        }
        else if (robot == tuw_robotino && runHandover_){
            kclhandGraspActionClient.sendGoal(graspServiceKCL.goal);
            kclhandGraspActionClient.waitForResult(ros::Duration(5.0));
            if (kclhandGraspActionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("(handover) (hand msg) HAND Close! \n");
                handover_success_ = true;
            }else{
                ROS_ERROR("(handover) (hand msg) FAILED to Close! \n");
                handover_success_ = false;
                handover_success_ = true; //to be removed after hadn is fixed
                //runHandover_ = false;
            }
        }
        this->resetSafety();


        //ROS_INFO("(handover) going to initial pose with the open hand \n");

        //if(runHandover_) robotinoQueue->jointPtp(start);


        ROS_INFO("(handover) going to the handover pose with the open hand \n");

        if(runHandover_)robotinoQueue->jointPtp(end);

        ROS_INFO("(handover) waiting to grasp the object \n");
        sleep(1.0);

        bool grasp_value = false; //detect object

        force_past.clear();
        torque_past.clear();
        while(runHandover_ && !grasp_value){

            if (record_magnitude(current_forces_, current_torques_)&&runHandover_)
            {
                grasp_value = detector_take();
            }
            lRate.sleep();
        }


        if(grasp_value && runHandover_){

            if (robot == uibk_robotino && runHandover_){

                if ( ros::service::call(HAND_SERVICE, graspService) ){
                    ROS_INFO("(handover) (hand msg) HAND Grasped! \n");
                    handover_success_ = true;
                }else{
                    ROS_ERROR("handover) (hand msg) FAILED to Graps! \n");
                    handover_success_ = false;
                    runHandover_ = false;
                }
            }
            else if (robot == tuw_robotino && runHandover_){
                kclhandGraspActionClient.sendGoal(graspServiceKCL.goal);
                kclhandGraspActionClient.waitForResult(ros::Duration(5.0));
                if (kclhandGraspActionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    ROS_INFO("(handover) (hand msg) HAND Grasped! \n");
                    handover_success_ = true;
                }else{
                    ROS_ERROR("handover) (hand msg) FAILED to Grap! \n");
                    handover_success_ = false;
                    handover_success_ = true; //to be removed after hand is fixed
                    //runHandover_ = false;
                }
            }
        }

        this->resetSafety();

        //ROS_INFO("(handover) going to initial pose with the closed hand");

        //if(runHandover_) robotinoQueue->jointPtp(start);
        sleep(0.2);
        ROS_INFO("(handover) cheking if object is in the hand \n");
        force_past.clear();
        torque_past.clear();
        k = 0;
        while (k < 3){
            record_magnitude_simple(current_forces_, current_torques_);
            lRate.sleep();
            k++;
        }
        double new_magnitude = getMean(force_past);
        if (abs((init_magnitude - new_magnitude)) < 0.2){
            ROS_ERROR("handover) FAILED to Graps. Empty hand! \n");
            handover_success_ = false;
            runHandover_ = false;
        }
        this->resetSafety();


    }
    else if(give.compare(goal->action_type.c_str()) ==0 && runHandover_){


       // ROS_INFO("(handover) going to the initial pose \n");

       // if (runHandover_) robotinoQueue->jointPtp(start);

        ROS_INFO("(handover) going to the handover pose with the closed hand");
        //stage = 6; // initial pose with the closed hand
        //cout << "(handover) current stage "<<stage<<endl;

        if (runHandover_) robotinoQueue->jointPtp(end);
        sleep(1.0);

        ROS_INFO("(handover) waiting to release the object \n");

        bool release = false;
        torque_past.clear();
        force_past.clear();

        double mean=0;
        bool ref = false;
        bool isCurPos = false;
        int condition=0;
        while (!release && runHandover_){

            record_magnitude_simple(current_forces_, current_torques_);
            //force_past is a global
            int i = force_past.size() - 1;
            if ( i <= MIN_VALS_GIVE - 1 )	//we enter here only once as part of the initialisation
            {
                mean = getMean(force_past);
                double absValRef = force_past[i] - mean;
                ref = absValRef>0 ? true : false;

            }
            else if (abs(force_past[i] - mean) > 0.05)
            {
                double absVal = force_past[i] - mean;
                isCurPos = (force_past[i - 1] - mean) >0 ? true : false;		//update current value
                if(isCurPos != ref)
                {
                    ++condition;
                    release = condition == 4? true : false;
                }
                else
                {
                    ref = -ref;				//here we change ref to the other sign
                    condition = 0;
                }

            }
            lRate.sleep();
        }

        if(release && runHandover_){

            if (robot == uibk_robotino && runHandover_){

                if ( ros::service::call(HAND_SERVICE, releaseService) ){
                    ROS_INFO("(handover) (hand msg) HAND Released!");
                    handover_success_ = true;
                }else{
                    ROS_ERROR("handover) (hand msg)  FAILED to Release!");
                    handover_success_ = false;
                    runHandover_ = false;
                }
            }
            else if (robot == tuw_robotino && runHandover_){
                kclhandGraspActionClient.sendGoal(releaseServiceKCL.goal);
                kclhandGraspActionClient.waitForResult(ros::Duration(5.0));
                if (kclhandGraspActionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    ROS_INFO("(handover) HAND Released!");
                    handover_success_ = true;
                }else{
                    ROS_ERROR("handover) FAILED to Release!");
                    handover_success_ = false;
                    handover_success_ = true; //to be removed after hadn is fixed
                    //runHandover_ = false;
                }
            }
        }
        this->resetSafety();

        ROS_INFO("(handover) going to initial pose with the open hand \n");
        if (runHandover_) robotinoQueue->jointPtp(start);

    }
    else{
        ROS_ERROR(" Handover: Handover action unknown \n");
        //cout<<endl;
        handover_success_ = false;
    }

    ROS_INFO(" Handover: Handover sequence finished. \n");

    //stop arm queue

     robotinoQueue->stopQueue();

    if (!runHandover_){
        handoverResult.result_status = "canceled";
        handoverServer.setAborted(handoverResult);
        ROS_INFO(" Handover: failed \n ");
        //cout<< endl;
    }
    else if(handover_success_){
        handoverResult.result_status = "success";
        handoverServer.setSucceeded(handoverResult);
        ROS_INFO(" Handover: Sucessfuly finished \n ");
        //cout<< endl;
    }
    else{
        handoverResult.result_status = "failure";
        handoverServer.setAborted(handoverResult);
        ROS_INFO(" Handover: failed \n");
        //cout<< endl;
    }
}

HandoverAction::~HandoverAction() {


}

void HandoverAction::sensorReadCallbackWrist(std_msgs::Float64MultiArray msg){
    wrist_sensor_values_ = msg.data;
    copy( wrist_sensor_values_.begin(), wrist_sensor_values_.begin() + 3, current_forces_.begin());
    copy( wrist_sensor_values_.begin() + 3, wrist_sensor_values_.end(), current_torques_.begin());

}

void HandoverAction::sensorReadCallbackFingers(std_msgs::Float64MultiArray msg){

    fingertip_sensor_values_ = msg.data;
}

bool HandoverAction::record_magnitude(const std::vector<double>& frc, const std::vector<double>& trq)
{
    double frc_mag = sqrt(pow(frc[X], 2) + pow(frc[Y], 2) + pow(frc[Z], 2));
    double trq_mag = sqrt(pow(trq[X], 2) + pow(trq[Y], 2) + pow(trq[Z], 2));

    int f_size = force_past.size();
    int t_size = torque_past.size();
    if (f_size < MIN_VALS)
    {
        force_past.push_back(frc_mag);
        ++f_size;
    }
    else
    {
        double tmp=force_past[MIN_VALS-1];
        force_past[MIN_VALS - 1] = frc_mag;			//last value
        for (int i = MIN_VALS - 2; i >= 0; --i)	//value before the last
        {
            std::swap(force_past[i],tmp);
        }
    }

    if (t_size < MIN_VALS)
    {
        torque_past.push_back(trq_mag);
        ++t_size;
    }
    else
    {
        double tmp = torque_past[MIN_VALS - 1];
        torque_past[MIN_VALS - 1] = trq_mag;			//last value
        for (int i = MIN_VALS - 2; i >= 0; --i)	//value before the last
        {
            std::swap(torque_past[i], tmp);
        }
    }

    if (t_size == MIN_VALS && f_size == MIN_VALS)	//if we acquired the right amount of data
    {
        return true;
    }
    return false;	//too few or too many values
}

bool HandoverAction::detector_take()
{
    //if we don't get what expected, fail
    assert(force_past.size() == MIN_VALS && torque_past.size() == MIN_VALS);

    std::vector<double> f_diffs;
    std::vector<double> t_diffs;

    int idx=force_past.size();

    //this can be optimised in a for loop if needed
    //NOTE here the order is reversed: v[0] is the newest - if the for loop is increasing, we will have the same order
    f_diffs.push_back(force_past.at(idx-1) - force_past.at(idx - 2));
    f_diffs.push_back(force_past.at(idx-2) - force_past.at(idx - 3));
    t_diffs.push_back(torque_past.at(idx-1) - torque_past.at(idx - 2));
    t_diffs.push_back(torque_past.at(idx-2) - torque_past.at(idx - 3));

    //normally we should always have exactly 2 values, if we don't something's gone wrong
    assert(f_diffs.size() == 2 && t_diffs.size() == 2);

    //newest - oldest diffs, true if the diff of the diff is either bigger than 1 or less than -1
    //according to the data value is 2.4

    bool f_good = ((f_diffs.at(1) - f_diffs.at(0)) > 0.9 || (f_diffs.at(1) - f_diffs.at(0)) < -0.9) ? true : false;
    bool t_good = ((t_diffs.at(1) - t_diffs.at(0)) > 0.9 || (t_diffs.at(1) - t_diffs.at(0)) < -0.9) ? true : false;

    return f_good;	//return true only if force threashold is good, torque is for future use
}

//returns true if enough data is collected and the detection algorithm can be executed, returns false is not enough values are collected
//in normal operating conditions, fills force and torque vectors by shifting older values towards the beginning of each vector (i.e. v[0] is the oldest)
void HandoverAction::record_magnitude_simple(const std::vector<double>& frc, const std::vector<double>& trq)
{

    double frc_mag = sqrt(pow(frc[X], 2) + pow(frc[Y], 2) + pow(frc[Z], 2));

    force_past.push_back(frc_mag);

}

//this method calculates the mean
double HandoverAction::getMean(const std::vector<double>& starters)
{
    int numVal=starters.size();

    double sum=0;
    for(unsigned int i=0;i<starters.size();++i)
    {
        sum+=starters[i];
    }

    return (sum/numVal);
}
void HandoverAction::preemptCB(){

    ROS_INFO("(handover) Canceled handover action by the high-level planner");
    cout<<endl;
    runHandover_ = false;

}

void HandoverAction::moveTilt(double val){
    std_msgs::Float64 tilt_msg;
    tilt_msg.data = val;
    tiltPub.publish(tilt_msg);
}

void HandoverAction::movePan(double val){
    std_msgs::Float64 pan_msg;
    pan_msg.data = val;
    panPub.publish(pan_msg);
}

void HandoverAction::resetSafety(){
    std_msgs::Bool reset;
    reset.data = true;
    safety_pub_.publish(reset);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "handover");
    sleep(1); ros::AsyncSpinner spinner(10); spinner.start();

    HandoverAction hadnover(HANDOVER_NAME);

    getchar();
    return 0;

}


