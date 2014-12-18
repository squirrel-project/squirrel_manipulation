class PickupAction {
protected:

    ros::NodeHandle nh;
    actionlib::SimpleActionServer<squirrel_manipulation_msgs::PickUpAction> as;
    std::string actionName;

    squirrel_manipulation_msgs::PickUpFeedback feedback;
    squirrel_manipulation_msgs::PickUpResult result;

public:

    PickupAction(std::string name);
    ~PickupAction();
    
    void executeCB(const squirrel_manipulation_msgs::PickUpGoalConstPtr &goal);


};
