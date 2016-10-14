#include <squirrel_handovers/handover.hpp>


using namespace std;

HandoverAction::HandoverAction(const std::string std_HandoverServerActionName) :
    handoverServer(nh, std_HandoverServerActionName, boost::bind(&HandoverAction::executeHandover, this, _1), false),
    private_nh("~")
{
    handoverServer.start();
    ROS_INFO("Handover: server started \n");

}

void HandoverAction::executeHandover(const squirrel_manipulation_msgs::HandoverGoalConstPtr &goal) {


    ROS_INFO("Handover: action started \n");
    cout<<endl;

    ros::Rate lRate(20.0);
    ros::spinOnce();

    while(true){
        
        lRate.sleep();
    }

    ROS_INFO(" Handover: Handover done.");
    cout<<endl;
    
    bool handover_success_;


    if(handover_success_){
        handoverResult.result_status = "success";
        handoverServer.setSucceeded(handoverResult);
        ROS_INFO(" Handover: Sucessfuly finished ");
        cout<< endl;
    }
    else{
        handoverResult.result_status = "failure";
        handoverServer.setAborted(handoverResult);
        ROS_INFO(" Handover: failed ");
        cout<< endl;}
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "manipulation");

    HandoverAction handover(HANDOVER_NAME);
    ros::spin();

    return 0;

}


