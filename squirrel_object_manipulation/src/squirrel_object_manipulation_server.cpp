#include "squirrel_object_manipulation/squirrel_object_manipulation_server.h"

using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
SquirrelObjectManipulationServer::SquirrelObjectManipulationServer ( ros::NodeHandle &n, const std::string &action_name ) :
	n_ ( new ros::NodeHandle(n) ),
	action_name_ ( action_name ),
    as_ ( *n_, action_name, boost::bind(&SquirrelObjectManipulationServer::actionServerCallBack, this, _1), false )
{
    as_.start();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
SquirrelObjectManipulationServer::~SquirrelObjectManipulationServer ()
{}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::initialize ()
{
    // Read the hand name
    string hand_name = METAHAND_STRING_;
    if ( !n_->getParam(addNodeName("hand"), hand_name) )
        ROS_WARN ( "[SquirrelObjectManipulationServer::initialize] No input hand name, using default '%s'", hand_name.c_str() );
    if ( hand_name.compare(METAHAND_STRING_) == 0 )
    {
        hand_type_ = SquirrelObjectManipulationServer::METAHAND;
        ROS_INFO ( "[SquirrelObjectManipulationServer::initialize] Metahand selected" );
    }
    else if ( hand_name.compare(SOFTHAND_STRING_) == 0 )
    {
        hand_type_ = SquirrelObjectManipulationServer::SOFTHAND;
        ROS_INFO ( "[SquirrelObjectManipulationServer::initialize] Softhand selected" );
    }
    else
    {
        hand_type_ = SquirrelObjectManipulationServer::UNKNOWN_HAND;
        ROS_ERROR ( "[SquirrelObjectManipulationServer::initialize] Unknown hand name '%s'", hand_name.c_str() );
        return false;
    }
    // Read the planning parameters
    planning_time_ = DEFAULT_PLANNING_TIME_;
    if ( !n_->getParam(addNodeName("planning_time"), planning_time_) )
        ROS_WARN ( "[SquirrelObjectManipulationServer::initialize] Planning time not given, using default value %.2f",
                    DEFAULT_PLANNING_TIME_ );
    plan_with_octomap_collisions_ = true;
    if ( !n_->getParam(addNodeName("plan_with_octomap_collisions"), plan_with_octomap_collisions_) )
    {
        ROS_WARN ( "[SquirrelObjectManipulationServer::initialize] Planning with octomap collisions not given, using default value TRUE" );
    }
    else
    {
        if ( !plan_with_octomap_collisions_ )
            ROS_WARN ( "[SquirrelObjectManipulationServer::initialize] Planning with octomap collisions is set to FALSE" );
    }
    plan_with_self_collisions_ = true;
    if ( !n_->getParam(addNodeName("plan_with_self_collisions"), plan_with_self_collisions_) )
    {
        ROS_WARN ( "[SquirrelObjectManipulationServer::initialize] Planning with self collision not given, using default value TRUE" );
    }
    else
    {
        if ( !plan_with_self_collisions_ )
            ROS_WARN ( "[SquirrelObjectManipulationServer::initialize] Planning with self collisions is set to FALSE" );
    }
    approach_height_ = DEFAULT_APPROACH_HEIGHT_;
    if ( !n_->getParam(addNodeName("approach_height"), approach_height_) )
        ROS_WARN ( "[SquirrelObjectManipulationServer::initialize] Approach not given, using default value %.2f",
                    DEFAULT_APPROACH_HEIGHT_ );

    ROS_INFO ( "[SquirrelObjectManipulationServer::initialize] Started with parameters:" );
    cout << "action_name = " << action_name_ << "\n"
         << "hand_name = " << hand_name << "\n"
         << "planning_time = " << planning_time_ << "\n"
         << "plan_with_octomap_collisions = " << plan_with_octomap_collisions_ << "\n"
         << "plan_with_self_collisions = " << plan_with_self_collisions_ << "\n"
         << "approach_height = " << approach_height_ << endl;

    // Read the placement parameter
    do_full_placement_ = false;
    if ( !n_->getParam(addNodeName("full_placement"), do_full_placement_) )
    {
        ROS_WARN ( "[SquirrelObjectManipulationServer::initialize] Do full placement not given, using default value FALSE" );
    }

    // Setup service clients
    arm_fold_client_ = new ros::ServiceClient ( n_->serviceClient<squirrel_motion_planner_msgs::FoldArm>("/squirrel_8dof_planning/fold_arm") );
    arm_unfold_client_ = new ros::ServiceClient ( n_->serviceClient<squirrel_motion_planner_msgs::UnfoldArm>("/squirrel_8dof_planning/unfold_arm") );
    arm_end_eff_planner_client_ = new ros::ServiceClient ( n_->serviceClient<squirrel_motion_planner_msgs::PlanEndEffector>("/squirrel_8dof_planning/find_plan_end_effector") );
    arm_pose_planner_client_ = new ros::ServiceClient ( n_->serviceClient<squirrel_motion_planner_msgs::PlanPose>("/squirrel_8dof_planning/find_plan_pose") );
    arm_send_trajectory_client_ = new ros::ServiceClient ( n_->serviceClient<squirrel_motion_planner_msgs::SendControlCommand>("/squirrel_8dof_planning/send_trajectory_controller") );
    examine_waypoint_client_ = new ros::ServiceClient ( n_->serviceClient<squirrel_waypoint_msgs::ExamineWaypoint>("/squirrel_perception_examine_waypoint") );
    create_octomap_client_ = new ros::ServiceClient ( n_->serviceClient<squirrel_object_perception_msgs::CreateOctomapWithLumps>("/squirrel_create_octomap_with_lumps") );

    // Setup action clients
    haf_client_ = new actionlib::SimpleActionClient<haf_grasping::CalcGraspPointsServerAction> ( "calc_grasppoints_svm_action_server", true );
    move_base_client_ = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ( "move_base", true );

    // Check that the hand is available
    hand_available_ = false;
    if ( hand_type_ == SquirrelObjectManipulationServer::METAHAND )
    {
        hand_client_ = new ros::ServiceClient ( n_->serviceClient<kclhand_control::HandOperationMode>("/hand_controller/hand_operation_mode") );
        hand_available_ = hand_client_->waitForExistence ( ros::Duration(3.0) );
        if ( !hand_available_ )
            ROS_WARN ( "[SquirrelObjectManipulationServer::initialize] KCL hand operation service unavailable, are you running in simulation?" );
    }
    else if ( hand_type_ == SquirrelObjectManipulationServer::SOFTHAND )
    {
        hand_client_ = new ros::ServiceClient ( n_->serviceClient<squirrel_manipulation_msgs::SoftHandGrasp>("/softhand_grasp") );
        hand_available_ = hand_client_->waitForExistence ( ros::Duration(3.0) );
        if ( !hand_available_ )
            ROS_WARN ( "[SquirrelObjectManipulationServer::initialize] SoftHand operation service unavailable, are you running in simulation?" );

    }

    // Get the folded and unfolded poses from the parameter server
    vector<double> loaded_poses;
    if ( !n_->getParam(addNodeName("trajectory_folding_arm"), loaded_poses) )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::initialize] Could not retrieve unfolded poses" );
        return false;
    }

    if ( loaded_poses.size() % NUM_ARM_JOINTS_ == 0 )
    {
        // Folded pose is the first 5 values
        folded_pose.resize ( NUM_ARM_JOINTS_ );
        for ( int i = 0; i < NUM_ARM_JOINTS_; ++i )
            folded_pose[i] = loaded_poses[i];
        // Unfolded pose is the last 5 values
        int num_vals = loaded_poses.size();
        unfolded_pose.resize ( NUM_ARM_JOINTS_ );
        cout << "Unfolded position: ";
        for ( int i = 0; i < NUM_ARM_JOINTS_; ++i )
        {
            unfolded_pose[i] = loaded_poses[num_vals-(NUM_ARM_JOINTS_-i)];
            cout << unfolded_pose[i] << " ";
        }
        cout << endl;
    }
    else
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::initialize] Parameter list 'trajectory_folding_arm' is not divisible by the number of arm joints, folding arm trajectory could not been loaded" );
        return false;
    }

    goal_pose_pub_ = n_->advertise<visualization_msgs::Marker> ( addNodeName("goal_pose"), 3, this );
    goal_marker_.header.frame_id = MAP_FRAME_;
    goal_marker_.ns = "end_effector_pose";
    goal_marker_.id = 0;
    goal_marker_.type = visualization_msgs::Marker::ARROW;
    goal_marker_.action = visualization_msgs::Marker::ADD;
    goal_marker_.lifetime = ros::Duration();
    goal_marker_.color.r = 1;
    goal_marker_.color.g = 0;
    goal_marker_.color.b = 0;
    goal_marker_.color.a = 1;
    goal_marker_.scale.x = 0.05;
    goal_marker_.scale.y = 0.0;
    goal_marker_.scale.z = 0.0;

    // To store the joint states and the trajectory command
    current_joints_.resize ( NUM_BASE_JOINTS_ + NUM_ARM_JOINTS_ );  // 5 for arm and 3 for base
    current_cmd_.resize ( NUM_BASE_JOINTS_ + NUM_ARM_JOINTS_ );
    joints_state_sub_ = n_->subscribe ( "/joint_states", 1, &SquirrelObjectManipulationServer::jointsStateCallBack, this );
    joints_command_sub_ = n_->subscribe ( "/arm_controller/joint_trajectory_controller/command", 1, &SquirrelObjectManipulationServer::jointsCommandCallBack, this );

    // Publish to the controller
    trajectory_controller_pub_ = n_->advertise<trajectory_msgs::JointTrajectory> ( "/arm_controller/joint_trajectory_controller/command", 10 );

    // Head controller
    head_pub_ = n_->advertise<std_msgs::Float64> ( "/head_controller/command", 3, this );

    // Store the transform between the hand and the wrist
    try
    {
        if ( tf_listener_.waitForTransform ( PLANNING_LINK_, GRASPING_LINK_, ros::Time(0), ros::Duration(5.0) ) )
        {
            tf_listener_.lookupTransform ( PLANNING_LINK_, GRASPING_LINK_, ros::Time(0), hand_to_wrist_transform_ );
        }
        else
        {
            ROS_ERROR ( "[SquirrelObjectManipulationServer::initialize] Error waiting for transform %s -> %s", PLANNING_LINK_, GRASPING_LINK_ );
            ros::Duration(1.0).sleep();
            return false;
        }
        ROS_INFO_STREAM ( "[SquirrelObjectManipulationServer::initialize] Found transform:\n - Translation: [" <<
                           hand_to_wrist_transform_.getOrigin().getX() << ", " << hand_to_wrist_transform_.getOrigin().getY() << ", " <<
                           hand_to_wrist_transform_.getOrigin().getZ() << "]\n - Rotation [" <<
                           hand_to_wrist_transform_.getRotation().getX() << ", " << hand_to_wrist_transform_.getRotation().getY() << ", " <<
                           hand_to_wrist_transform_.getRotation().getZ() << ", " << hand_to_wrist_transform_.getRotation().getW() << "]\n" );
    }
    catch ( tf::TransformException ex )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::initialize] %s", ex.what() );
        ROS_ERROR ( "[SquirrelObjectManipulationServer::initialize] Error looking up transform %s -> %s", PLANNING_LINK_, GRASPING_LINK_ );
        ros::Duration(1.0).sleep();
        return false;
    }

    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SquirrelObjectManipulationServer::actionServerCallBack ( const squirrel_manipulation_msgs::ManipulationGoalConstPtr &goal )
{
    ROS_INFO ( "[SquirrelObjectManipulationServer::actionServerCallBack] Started" );
    std::cout << *goal << std::endl;

    // Checking the command (TODO: change the action definition to have action type as input, right now can exploit the obejct_id field)
    feedback_.current_phase = "checking input command";
    publishFeedback ( "started" );
    action_type_ = SquirrelObjectManipulationServer::UNKNOWN_ACTION;
    if ( goal->manipulation_type.compare("open") == 0 || goal->manipulation_type.compare("open hand") == 0 ) action_type_ = SquirrelObjectManipulationServer::OPEN_HAND_ACTION;
    else if ( goal->manipulation_type.compare("close") == 0 || goal->manipulation_type.compare("close hand") == 0 ) action_type_ = SquirrelObjectManipulationServer::CLOSE_HAND_ACTION;
    else if ( goal->manipulation_type.compare("joints") == 0 || goal->manipulation_type.compare("move joints") == 0 ) action_type_ = SquirrelObjectManipulationServer::JOINTS;
    else if ( goal->manipulation_type.compare("cartesian") == 0 || goal->manipulation_type.compare("move cartesian") == 0 ) action_type_ = SquirrelObjectManipulationServer::CARTESIAN;
    else if ( goal->manipulation_type.compare("grasp") == 0 || goal->manipulation_type.compare("grasp object") == 0 ) action_type_ = SquirrelObjectManipulationServer::GRASP;
    else if ( goal->manipulation_type.compare("drop") == 0 || goal->manipulation_type.compare("drop object") == 0 ) action_type_ = SquirrelObjectManipulationServer::DROP;
    else if ( goal->manipulation_type.compare("pick") == 0 || goal->manipulation_type.compare("pick object") == 0 ) action_type_ = SquirrelObjectManipulationServer::PICK;
    else if ( goal->manipulation_type.compare("place") == 0 || goal->manipulation_type.compare("place object") == 0 ) action_type_ = SquirrelObjectManipulationServer::PLACE;
    else if ( goal->manipulation_type.compare("haf grasp") == 0 || goal->manipulation_type.compare("haf grasp object") == 0 ) action_type_ = SquirrelObjectManipulationServer::HAF_GRASP;
    else if ( goal->manipulation_type.compare("haf pick") == 0 || goal->manipulation_type.compare("haf pick object") == 0 ) action_type_ = SquirrelObjectManipulationServer::HAF_PICK;
    else if ( goal->manipulation_type.compare("haf grasp ful") == 0 ) action_type_ = SquirrelObjectManipulationServer::HAF_GRASP_FULL;
    else if ( goal->manipulation_type.compare("haf pick full") == 0 ) action_type_ = SquirrelObjectManipulationServer::HAF_PICK_FULL;
    else if ( goal->manipulation_type.compare("fold") == 0 || goal->manipulation_type.compare("fold arm") == 0 ) action_type_ = SquirrelObjectManipulationServer::FOLD_ARM_ACTION;
    else if ( goal->manipulation_type.compare("unfold") == 0 || goal->manipulation_type.compare("unfold arm") == 0 ) action_type_ = SquirrelObjectManipulationServer::UNFOLD_ARM_ACTION;
    else if ( goal->manipulation_type.compare("prepare haf") == 0 ) action_type_ = SquirrelObjectManipulationServer::PREPARE_FOR_HAF;
    else if ( goal->manipulation_type.compare("prepare recognition") == 0 ) action_type_ = SquirrelObjectManipulationServer::PREPARE_FOR_RECOGNITION;
    else if ( goal->manipulation_type.compare("previous") == 0 ) action_type_ = SquirrelObjectManipulationServer::RETURN_TO_PREVIOUS;
    if ( action_type_ == SquirrelObjectManipulationServer::UNKNOWN_ACTION )
    {
        ROS_WARN ( "[SquirrelObjectManipulationServer::actionServerCallBack] Could not interpret input command '%s'", goal->manipulation_type.c_str() );
        publishFeedback ( "failed" );
        as_.setAborted ( result_ );
        return;
    }
    ROS_WARN ( "[SquirrelObjectManipulationServer::actionServerCallBack] Requested action '%s'", goal->manipulation_type.c_str() );
    publishFeedback ( "success" );

    /*
    // Check if the arm is folded
    feedback_.current_phase = "checking arm configuration";
    publishFeedback ( "started" );
    if ( armIsFolded() && action_type_ != SquirrelObjectManipulationServer::UNFOLD_ARM_ACTION )
    {
        ROS_WARN ( "[SquirrelObjectManipulationServer::actionServerCallBack] Arm is folded, it must be unfolded first before manipulating objects" );
        publishFeedback ( "failed" );
        as_.setAborted ( result_ );
        return;
    }
    publishFeedback ( "success" );
    */

    // Check that a valid object height is given (otherwise the graps might be too low and collide with the ground)
    feedback_.current_phase = "checking object bounding cylinder";
    publishFeedback ( "started" );
    if ( (action_type_ == SquirrelObjectManipulationServer::GRASP ||
          action_type_ == SquirrelObjectManipulationServer::PICK ||
          action_type_ == SquirrelObjectManipulationServer::DROP ||
          action_type_ == SquirrelObjectManipulationServer::PLACE ||
          action_type_ == SquirrelObjectManipulationServer::HAF_GRASP ||
          action_type_ == SquirrelObjectManipulationServer::HAF_PICK ||
          action_type_ == SquirrelObjectManipulationServer::HAF_GRASP_FULL ||
          action_type_ == SquirrelObjectManipulationServer::HAF_PICK_FULL) &&
         goal->object_bounding_cylinder.height <= 0 )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::actionServerCallBack] No object bounding cylinder given" );
        publishFeedback ( "failed" );
        as_.setAborted ( result_ );
        return;
    }
    publishFeedback ( "success" );

    // Clear the list of lumps when creating octomap
    bool create_octomap = false;
    create_octomap_goal_.request.lumps.clear();
    create_octomap_goal_.request.lumps.resize ( 1 );
    create_octomap_goal_.request.lumps[0].header.frame_id = MAP_FRAME_;
    create_octomap_goal_.request.lumps[0].pose = goal->pose.pose;
    create_octomap_goal_.request.lumps[0].bounding_cylinder = goal->object_bounding_cylinder;

    // If a valid scene object is found then create a grasp goal according to the object's properties
    feedback_.current_phase = "retrieving scene object properties";
    publishFeedback ( "started" );
    recognition_cloud_.data.clear();
    squirrel_manipulation_msgs::ManipulationGoalPtr object_goal = boost::make_shared<squirrel_manipulation_msgs::ManipulationGoal>();
    if ( action_type_ == SquirrelObjectManipulationServer::GRASP ||
         action_type_ == SquirrelObjectManipulationServer::PICK ||
         action_type_ == SquirrelObjectManipulationServer::HAF_GRASP ||
         action_type_ == SquirrelObjectManipulationServer::HAF_PICK ||
         action_type_ == SquirrelObjectManipulationServer::HAF_GRASP_FULL ||
         action_type_ == SquirrelObjectManipulationServer::HAF_PICK_FULL )
    {
        // Set creating the octomap to true
        create_octomap = true;

        // Copy the message
        object_goal->manipulation_type = goal->manipulation_type;
        object_goal->object_id = goal->object_id;
        object_goal->pose = goal->pose;
        object_goal->object_bounding_cylinder = goal->object_bounding_cylinder;
        object_goal->joints = goal->joints;
        // Get a scene object from the message store
        squirrel_object_perception_msgs::SceneObject scene_object;
        if ( getSceneObject(object_goal->object_id, scene_object) )
        {
            // Copy scene object information to object goal
            object_goal->pose.header.frame_id = scene_object.header.frame_id;
            object_goal->object_bounding_cylinder = scene_object.bounding_cylinder;
            recognition_cloud_ = scene_object.cloud;
            std::cout << "Scene object cloud has size " << scene_object.cloud.data.size() << std::endl;
            // If no frame id specified then assume it is map
            if ( object_goal->pose.header.frame_id.empty() )
            {
                ROS_WARN ( "[SquirrelObjectManipulationServer::actionServerCallBack] Message strore object has no frame id specified, reverting to map" );
                object_goal->pose.header.frame_id = MAP_FRAME_;
                object_goal->pose.pose = scene_object.pose;
            }
            // Otherwise, transform the pose to map
            else
            {
                geometry_msgs::PoseStamped scene_object_pose, transformed_pose;
                scene_object_pose.header.frame_id = object_goal->pose.header.frame_id;
                scene_object_pose.pose.position = scene_object.pose.position;
                scene_object_pose.pose.orientation.w = 1.0;
                transformPose ( object_goal->pose.header.frame_id, MAP_FRAME_, scene_object_pose, transformed_pose );
                object_goal->pose.pose.position = transformed_pose.pose.position;
            }
            // TODO: scene object specifies only x and y, and assumes object is on the floor (i.e. z = 0)
            //object_goal->pose.pose.position.z = object_goal->object_bounding_cylinder.height/2.0;
            create_octomap_goal_.request.lumps[0].pose = object_goal->pose.pose;
            create_octomap_goal_.request.lumps[0].bounding_cylinder = object_goal->object_bounding_cylinder;
        }
    }
    publishFeedback ( "success" );

    // Add the boxes to 8dof planning octomap
    std::vector<std::string> box_names;
    box_names.push_back ( "box1_location" );
    box_names.push_back ( "box2_location" );
    box_names.push_back ( "box3_location" );
    for ( size_t i = 0; i < box_names.size(); ++i )
    {
        squirrel_object_perception_msgs::SceneObject box;
        if ( getSceneObject(box_names[i], box) )
        {
            create_octomap_goal_.request.lumps.push_back ( box );
            create_octomap_goal_.request.lumps.back().bounding_cylinder.height = 0.2;
            create_octomap_goal_.request.lumps.back().bounding_cylinder.diameter = 0.33;
        }
        else
        {
            ROS_WARN ( "[SquirrelObjectManipulationServer::actionServerCallBack] Could not find '%s' in message store", box_names[i].c_str() );
        }
    }

    // Create the octomap
    feedback_.current_phase = "creating octomap with lumps";
    publishFeedback ( "started" );
    if ( action_type_ == SquirrelObjectManipulationServer::DROP ||
         action_type_ == SquirrelObjectManipulationServer::PLACE )
    {
        create_octomap = true;
        create_octomap_goal_.request.lumps[0].bounding_cylinder.diameter = 0.4;
        create_octomap_goal_.request.lumps[0].bounding_cylinder.height = 0.15;
        //ROS_WARN ( "[SquirrelObjectManipulationServer::actionServerCallBack] Define lump from actual box location" );
    }
    if ( create_octomap )
    {
        if ( !create_octomap_client_->call(create_octomap_goal_) )
        {
            ROS_WARN ( "[SquirrelObjectManipulationServer::actionServerCallBack] Create octomap with lumps failed (if 8dof planning fails, set octomap service topic to /octomap_full in the config file)" );
        }
    }
    publishFeedback ( "success" );

    // Store the joint states in previous_joints_
    if ( action_type_ != SquirrelObjectManipulationServer::RETURN_TO_PREVIOUS )
        previous_joints_ = current_joints_;

    // Call the appropriate action
    bool success = false;
    if ( action_type_ == SquirrelObjectManipulationServer::OPEN_HAND_ACTION ) success = actuateHand ( SquirrelObjectManipulationServer::OPEN );
    else if ( action_type_ == SquirrelObjectManipulationServer::CLOSE_HAND_ACTION ) success = actuateHand ( SquirrelObjectManipulationServer::CLOSE );
    else if ( action_type_ == SquirrelObjectManipulationServer::JOINTS ) success = moveArmJoints ( goal->joints, "action request" );
    else if ( action_type_ == SquirrelObjectManipulationServer::CARTESIAN ) success = moveArmCartesian ( goal->pose, "action request" );
    else if ( action_type_ == SquirrelObjectManipulationServer::GRASP ) success = grasp ( object_goal );
    else if ( action_type_ == SquirrelObjectManipulationServer::DROP ) success = drop ( goal );
    else if ( action_type_ == SquirrelObjectManipulationServer::PICK ) success = pick ( object_goal );
    else if ( action_type_ == SquirrelObjectManipulationServer::PLACE ) success = place ( goal );
    else if ( action_type_ == SquirrelObjectManipulationServer::HAF_GRASP ) success = hafGrasp ( goal );
    else if ( action_type_ == SquirrelObjectManipulationServer::HAF_PICK ) success = hafPick ( goal );
    else if ( action_type_ == SquirrelObjectManipulationServer::HAF_GRASP_FULL ) success = hafGraspFull ( goal );
    else if ( action_type_ == SquirrelObjectManipulationServer::HAF_PICK_FULL ) success = hafPickFull ( goal );
    else if ( action_type_ == SquirrelObjectManipulationServer::FOLD_ARM_ACTION ) success = foldArm ();
    else if ( action_type_ == SquirrelObjectManipulationServer::UNFOLD_ARM_ACTION ) success = unfoldArm ();
    else if ( action_type_ == SquirrelObjectManipulationServer::PREPARE_FOR_HAF ) success = prepareForHafGrasping ( goal, HAF_MIN_DIST_ );
    else if ( action_type_ == SquirrelObjectManipulationServer::PREPARE_FOR_RECOGNITION ) success = prepareForHafGrasping ( goal );
    else if ( action_type_ == SquirrelObjectManipulationServer::RETURN_TO_PREVIOUS )
    {
        // Check that the previous joints are valid
        if ( previous_joints_.size() != current_joints_.size() )
        {
            ROS_ERROR ( "[SquirrelObjectManipulationServer::actionServerCallBack] Previous joints has size %lu, but expected %lu",
                        previous_joints_.size(), current_joints_.size() );
            success = false;
        }
        else
        {
            // Move joints
            success = moveArmJoints ( previous_joints_, "return" );
        }
    }

     if ( (action_type_ == SquirrelObjectManipulationServer::GRASP ||
           action_type_ == SquirrelObjectManipulationServer::PICK ||
           action_type_ == SquirrelObjectManipulationServer::DROP ||
           action_type_ == SquirrelObjectManipulationServer::PLACE ||
           action_type_ == SquirrelObjectManipulationServer::HAF_GRASP ||
           action_type_ == SquirrelObjectManipulationServer::HAF_PICK ||
           action_type_ == SquirrelObjectManipulationServer::HAF_GRASP_FULL ||
           action_type_ == SquirrelObjectManipulationServer::HAF_PICK_FULL) )
    {
        ROS_WARN ( "[SquirrelObjectManipulationServer::actionServerCallBack] Sleeping for 5 seconds" );
        ros::Duration(5.0).sleep();
    }

    // Return head to center
    moveHead ();

    if ( success )
    {
        ROS_INFO ( "[SquirrelObjectManipulationServer::actionServerCallBack] Succeeded!" );
        // Set the action state to succeeded
        as_.setSucceeded ( result_ );
    }
    else
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::actionServerCallBack] Failed!" );
        // Set the action state to aborted
        as_.setAborted ( result_ );
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::actuateHand ( const HandActuation &hand_actuation )
{
    if ( hand_type_ == SquirrelObjectManipulationServer::METAHAND ) return actuateMetahand ( hand_actuation );
    else if ( hand_type_ == SquirrelObjectManipulationServer::SOFTHAND ) return actuateSofthand ( hand_actuation);
    else ROS_ERROR ( "[SquirrelObjectManipulationServer::actuateHand] Unrecognized hand type" );

    // If made it here then did not reconize the actuation type
    return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::actuateMetahand ( const HandActuation &hand_actuation )
{
    ROS_INFO ( "[SquirrelObjectManipulationServer::actuateMetahand] Started" );

    // Set the phase of the action
    if ( hand_actuation == SquirrelObjectManipulationServer::OPEN )
    {
        feedback_.current_phase = "opening hand";
        metahand_goal_.request.operation_mode = OPEN_METAHAND_OPERATION_MODE;
    }
    else if ( hand_actuation == SquirrelObjectManipulationServer::CLOSE )
    {
        feedback_.current_phase = "closing hand";
        metahand_goal_.request.operation_mode = CLOSE_METAHAND_OPERATION_MODE;
    }
    else if ( hand_actuation == SquirrelObjectManipulationServer::FOLD_HAND )
    {
        feedback_.current_phase = "folding hand";
        metahand_goal_.request.operation_mode = FOLD_METAHAND_OPERATION_MODE;
    }
    else if ( hand_actuation == SquirrelObjectManipulationServer::CHANGE_WORKSPACE )
    {
        feedback_.current_phase = "changing workspace";
        metahand_goal_.request.operation_mode = CHANGE_WORKSPACE_METAHAND_OPERATION_MODE;
    }
    else
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::actuateMetahand] Unrecognized command" );
        feedback_.current_status = "failed";
        return false;
    }
    publishFeedback ( "started" );

    // Can only actuate the hand if the hand service is available
    if ( hand_available_ )
    {
        // Send the actuation command
        publishFeedback ( "commanding" );
        if ( !hand_client_->call(metahand_goal_) )
        {
            ROS_ERROR ( "[SquirrelObjectManipulationServer::actuateMetahand] Failed to call service" );
            publishFeedback ( "failed" );
            return false;
        }
        // Check the result
        if ( !metahand_goal_.response.result )
        {
            ROS_ERROR ( "[SquirrelObjectManipulationServer::actuateMetahand] Service returned result FALSE" );
            publishFeedback ( "failed" );
            return false;
        }
    }
    else
    {
        ROS_WARN ( "[SquirrelObjectManipulationServer::actuateMetahand] Cannot actuate hand because service is unavailable" );
        publishFeedback ( "failed" );
        // return false; // If it returns false then simulations won't work
    }

    // Successfully actuated the metahand
    ROS_INFO ( "[SquirrelObjectManipulationServer::actuateMetahand] Success!" );
    publishFeedback ( "success" );
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::actuateSofthand ( const HandActuation &hand_actuation )
{
    ROS_INFO ( "[SquirrelObjectManipulationServer::actuateSofthand] Started" );

    if ( hand_actuation == SquirrelObjectManipulationServer::OPEN )
    {
        feedback_.current_phase = "opening hand";
        softhand_goal_.request.position = OPEN_SOFTHAND_VALUE;
    }
    else if ( hand_actuation == SquirrelObjectManipulationServer::CLOSE )
    {
        feedback_.current_phase = "closing hand";
        softhand_goal_.request.position = CLOSE_SOFTHAND_VALUE;
    }
    else if ( hand_actuation == SquirrelObjectManipulationServer::FOLD_HAND )
    {
        // TODO: Philipp Zech...
        feedback_.current_phase = "folding hand";
        ROS_WARN ( "[SquirrelObjectManipulationServer::actuateSofthand] Folding hand is not yet implemented" );
        return false;
        // ...
    }
    else
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::actuateSofthand] Unrecognized command" );
        publishFeedback ( "failed" );
        return false;
    }

    publishFeedback ( "started" );

    // Can only actuate the hand if the hand service is available
    if ( hand_available_ )
    {
        // Send the actuation command
        publishFeedback ( "commanding" );
        if ( !hand_client_->call(softhand_goal_) )
        {
            ROS_ERROR ( "[SquirrelObjectManipulationServer::actuateSofthand] Failed to call service" );
            publishFeedback ( "failed" );
            return false;
        }
        // Check the result
        if ( !softhand_goal_.response.success )
        {
            ROS_ERROR ( "[SquirrelObjectManipulationServer::actuateSofthand] Service returned result FALSE" );
            publishFeedback ( "failed" );
            return false;
       }
    }
    else
    {
        ROS_WARN ( "[SquirrelObjectManipulationServer::actuateSofthand] Cannot actuate hand because service is unavailable" );
        publishFeedback ( "failed" );
        // return false; // If it returns false then simulations won't work
    }

    // Successfully actuated the softhand
    ROS_INFO ( "[SquirrelObjectManipulationServer::actuateSofthand] Success!" );
    publishFeedback ( "success" );
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::foldArm ()
{
    ROS_INFO ( "[SquirrelObjectManipulationServer::foldArm] Started" );

    // Check that the arm is not already folded
    if ( armIsFolded() )
    {
        ROS_WARN ( "[SquirrelGraspServer::foldArm] Arm is already folded" );
        return true;
    }

    // ***
    // Fold hand
    // ***
    if ( !actuateHand(SquirrelObjectManipulationServer::FOLD_HAND) )
    {
        ROS_ERROR ( "[SquirrelGraspServer::foldArm] Failed to fold hand" );
        return false;
    }

    // ***
    // Fold arm
    // ***
    feedback_.current_phase = "folding";
    publishFeedback ( "started" );
    fold_goal_.request.check_octomap_collision = false;//plan_with_octomap_collisions_;
    fold_goal_.request.check_self_collision = false; // Overriding ros parameter
    if ( !arm_fold_client_->call(fold_goal_) )
    {
        ROS_ERROR ( "[SquirrelGraspServer::foldArm] Failed to call service to fold the arm" );
        publishFeedback ( "failed" );
        return false;
    }
    // Sleep
    ros::Duration(1.0).sleep();
    // Wait for trajectory to finish
    publishFeedback ( "waiting for completion" );
    if ( !waitForTrajectoryCompletion() )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::foldArm] Trajectory to folded pose did not complete in time" );
        publishFeedback ( "failed" );
        return false;
    }
    // If not a successful result
    if ( fold_goal_.response.result != 0 )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::foldArm] Sending command to move to folded pose returned with result %i",
                    fold_goal_.response.result );
        publishFeedback ( "failed" );
        return false;
    }

    // Successfully commanded the arm
    ROS_INFO ( "[SquirrelObjectManipulationServer::foldArm] Sending command to move to folded pose returned with 0" );
    publishFeedback ( "success" );
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::unfoldArm ()
{
    ROS_INFO ( "[SquirrelObjectManipulationServer::unfoldArm] Started" );

    // Check that the arm is not already unfolded
    if ( armIsUnfolded() )
    {
        ROS_WARN ( "[SquirrelGraspServer::unfoldArm] Arm is already unfolded" );
        return true;
    }

    // ***
    // Unfold arm
    // ***
    feedback_.current_phase = "unfolding";
    publishFeedback ( "started" );
    unfold_goal_.request.check_octomap_collision = false;//plan_with_octomap_collisions_;
    unfold_goal_.request.check_self_collision = false;  // Overriding ros parameter
    if ( !arm_unfold_client_->call(unfold_goal_) )
    {
        ROS_ERROR ( "[SquirrelGraspServer::unfoldArm] Failed to call service to unfold the arm" );
        publishFeedback ( "failed" );
        return false;
    }
    // Sleep
    ros::Duration(1.0).sleep();
    // Wait for trajectory to finish
    publishFeedback ( "waiting for completion" );
    if ( !waitForTrajectoryCompletion() )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::unfoldArm] Trajectory to unfolded pose did not complete in time" );
        publishFeedback ( "failed" );
        return false;
    }
    // If not a successful result
    if ( unfold_goal_.response.result != 0 )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::unfoldArm] Sending command to move to unfolded pose returned with result %i",
                    unfold_goal_.response.result );
        publishFeedback ( "failed" );
        return false;
    }

    // ***
    // Change hand configuration
    // ***
    publishFeedback ( "changing hand configuration" );
    if ( !actuateHand(SquirrelObjectManipulationServer::CHANGE_WORKSPACE) )
    {
        ROS_ERROR ( "[SquirrelGraspServer::unfoldArm] Failed to change to upper workspace" );
        return false;
    }

    // Successfully commanded the arm
    ROS_INFO ( "[SquirrelObjectManipulationServer::unfoldArm] Sending command to move to unfolded pose returned with 0" );
    publishFeedback ( "success" );
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::grasp ( const squirrel_manipulation_msgs::ManipulationGoalConstPtr &goal )
{
    ROS_INFO ( "[SquirrelObjectManipulationServer::grasp] Started" );
    
    
    // Get the grasp and approach poses
    geometry_msgs::PoseStamped grasp_goal, approach_goal;
    if ( !getGripperAndApproachPose(goal, grasp_goal, approach_goal) )
    {
        ROS_ERROR ( "[SquirrelGraspServer::grasp] Could not get grasp and approach poses" );
        return false;
    }

    // Check if the arm needs to be unfolded
    if ( armIsFolded() )
    {
        // Unfold the arm
        if ( !unfoldArm() )
        {
            ROS_ERROR ( "[SquirrelGraspServer::grasp] Failed to unfold arm" );
            return false;
        }
    }

    // ***
    // Open hand
    // ***
    if ( !actuateHand(SquirrelObjectManipulationServer::OPEN) )
    {
        ROS_ERROR ( "[SquirrelGraspServer::grasp] Failed to open hand" );
        return false;
    }

    // ***
    // Approach
    // ***
    if ( !moveArmCartesian(approach_goal, STR_APPROACH_, APPROACH_TOLERANCE_) )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::grasp] Failed to reach approach pose" );
        return false;
    }

    // ***
    // Grasp
    // ***
    if ( !moveArmCartesian(grasp_goal, STR_GRASP_, APPROACH_TOLERANCE_) )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::grasp] Failed to reach grasp pose" );
        return false;
    }

    // ***
    // Close hand
    // ***
    ros::Duration(2.0).sleep();
    if ( !actuateHand(SquirrelObjectManipulationServer::CLOSE) )
    {
        ROS_ERROR ( "[SquirrelGraspServer::grasp] Failed to close hand" );
        //return false;
    }

    // Successfully grasped object!
    ROS_INFO ( "[SquirrelObjectManipulationServer::grasp] Success!" );
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::drop ( const squirrel_manipulation_msgs::ManipulationGoalConstPtr &goal )
{
    ROS_INFO ( "[SquirrelObjectManipulationServer::drop] Started" );

    // Get the drop and approach poses
    geometry_msgs::PoseStamped drop_goal, approach_goal;
    if ( !getGripperAndApproachPose(goal, drop_goal, approach_goal) )
    {
        ROS_ERROR ( "[SquirrelGraspServer::drop] Could not get drop and approach poses" );
        return false;
    }

    // ***
    // Approach
    // ***
    if ( !moveArmCartesian(approach_goal, STR_APPROACH_, APPROACH_TOLERANCE_) )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::drop] Failed to reach approach pose" );
        return false;
    }

    // ***
    // Drop
    // ***
    /*
    if ( !moveArmCartesian(drop_goal, "drop") )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::drop] Failed to reach drop pose" );
        end_eff_goal_.request.min_distance_before_folding = 0.0;
        return false;
    }
    */

    // ***
    // Open hand
    // ***
    ros::Duration(0.5).sleep();
    if ( !actuateHand(SquirrelObjectManipulationServer::OPEN) )
    {
        ROS_ERROR ( "[SquirrelGraspServer::drop] Failed to open hand" );
        return false;
    }

    // Successfully dropped object!
    ROS_INFO ( "[SquirrelObjectManipulationServer::drop] Success!" );
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::pick ( const squirrel_manipulation_msgs::ManipulationGoalConstPtr &goal )
{
    ROS_INFO ( "[SquirrelObjectManipulationServer::pick] Started" );

    // Get the pick up and approach poses
    geometry_msgs::PoseStamped pick_goal, approach_goal;
    if ( !getGripperAndApproachPose(goal, pick_goal, approach_goal) )
    {
        ROS_ERROR ( "[SquirrelGraspServer::pick] Could not get pick and approach poses" );
        return false;
    }

    // Check if the arm needs to be unfolded
    if ( armIsFolded() )
    {
        // Unfold the arm
        if ( !unfoldArm() )
        {
            ROS_ERROR ( "[SquirrelGraspServer::pick] Failed to unfold arm" );
            return false;
        }
    }

    // ***
    // Open hand
    // ***
    if ( !actuateHand(SquirrelObjectManipulationServer::OPEN) )
    {
        ROS_ERROR ( "[SquirrelGraspServer::pick] Failed to open hand" );
        return false;
    }

    // ***
    // Approach
    // ***
    if ( !moveArmCartesian(approach_goal, STR_APPROACH_, APPROACH_TOLERANCE_) )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::pick] Failed to reach approach pose" );
        return false;
    }

    // ***
    // Pick
    // ***
    moveHead ( HEAD_TO_HAND_ );
    if ( !moveArmCartesian(pick_goal, STR_PICK_, APPROACH_TOLERANCE_) )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::pick] Failed to reach pick up pose" );
        return false;
    }

    // ***
    // Close hand
    // ***
    ros::Duration(2.0).sleep();
    if ( !actuateHand(SquirrelObjectManipulationServer::CLOSE) )
    {
        ROS_ERROR ( "[SquirrelGraspServer::pick] Failed to close hand" );
        //return false;
    }

    // ***
    // Retract arm from grasp
    // ***
    ros::Duration(1.0).sleep();
    if ( !moveArmCartesian(approach_goal, STR_RETRACT_) )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::pick] Failed to reach retract pose" );
        return false;
    }
    
    // ***
    // Position arm for carrying
    // ***
    ros::Duration(1.0).sleep();
    std::vector<double> carry_pose = current_joints_;
    carry_pose[6] = 1.4;  // Rotate arm_joint4 upwards
    if ( !moveArmJoints(carry_pose, STR_CARRY_) )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::pick] Failed to reach carry pose" );
        return false;
    }
    moveHead ();

    // ***
    // Verify if pick was successful
    // ***
    /*
    ROS_WARN ( "[SquirrelObjectManipulationServer::pick] * * * Enter 'y' for success * * *" );
    char key_input;
    std::cin >> key_input;
    if ( key_input == 'y' || key_input == 'Y' )
    {
        // Successfully picked up object!
        ROS_INFO ( "[SquirrelObjectManipulationServer::pick] Success!" );
        return true;
    }
    else
    {
        // Did not successfully pick up object!
        ROS_ERROR ( "[SquirrelObjectManipulationServer::pick] Failure!" );
        return false;
    }

    ROS_ERROR ( "[SquirrelObjectManipulationServer::pick] Should not reach this line of code!" );
    return false;
    */

    // Successfully picked up object!
    ROS_INFO ( "[SquirrelObjectManipulationServer::pick] Success!" );
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::place ( const squirrel_manipulation_msgs::ManipulationGoalConstPtr &goal )
{
    ROS_INFO ( "[SquirrelObjectManipulationServer::place] Started" );

    // Get the place and approach poses
    geometry_msgs::PoseStamped place_goal, approach_goal;
    if ( !getGripperAndApproachPose(goal, place_goal, approach_goal) )
    {
        ROS_ERROR ( "[SquirrelGraspServer::place] Could not get place and approach poses" );
        return false;
    }

    // ***
    // Re-position arm from carrying
    // ***
    // TODO: Need to verify that this action should be applied!
    // Only do it if pick was called before (check arm values to make sure the rotation can be done?)
    ros::Duration(1.0).sleep();
    std::vector<double> carry_pose = current_joints_;
    carry_pose[6] = -1.4;  // Rotate arm_joint4 downwards
    if ( !moveArmJoints(carry_pose, STR_CARRY_) )
        ROS_WARN ( "[SquirrelObjectManipulationServer::place] Failed to return from carry pose" );

    // ***
    // Approach
    // ***
    if ( !moveArmCartesian(approach_goal, STR_APPROACH_, APPROACH_TOLERANCE_) )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::place] Failed to reach approach pose" );
        return false;
    }

    // ***
    // Place
    // ***
    moveHead ( HEAD_TO_HAND_ );
    if ( do_full_placement_ )
    {
        std::cout << "Place" << std::endl;
        if ( !moveArmCartesian(place_goal, STR_PLACE_) )
        {
            ROS_ERROR ( "[SquirrelObjectManipulationServer::place] Failed to reach place pose" );
            return false;
        }
    }

    // ***
    // Open hand
    // ***
    ros::Duration(0.5).sleep();
    if ( !actuateHand(SquirrelObjectManipulationServer::OPEN) )
    {
        ROS_ERROR ( "[SquirrelGraspServer::place] Failed to open hand" );
        //return false;
    }

    // ***
    // Retract arm
    // ***
    if ( do_full_placement_ )
    {
        ros::Duration(1.0).sleep();
        if ( !moveArmCartesian(approach_goal, STR_RETRACT_) )
        {
            ROS_ERROR ( "[SquirrelObjectManipulationServer::place] Failed to reach retract pose" );
            return false;
        }
    }

    // ***
    // Close hand for stability
    // ***
    ros::Duration(0.5).sleep();
    if ( !actuateHand(SquirrelObjectManipulationServer::CLOSE) )
    {
        ROS_ERROR ( "[SquirrelGraspServer::place] Failed to close hand" );
    }

    // ***
    // Position arm for navigating
    // ***
    ros::Duration(1.0).sleep();
    carry_pose = current_joints_;
    carry_pose[6] = 1.4;  // Rotate arm_joint4 upwards
    if ( !moveArmJoints(carry_pose, STR_CARRY_) )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::place] Failed to reach pose for navigation" );
        return false;
    }
    moveHead ();

    // Successfully placed object!
    ROS_WARN ( "[SquirrelObjectManipulationServer::place] Success!" );
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::hafGrasp ( const squirrel_manipulation_msgs::ManipulationGoalConstPtr &goal )
{
    ROS_INFO ( "[SquirrelObjectManipulationServer::hafGrasp] Started" );

    // ***
    // Grasp (haf calculation taken care of inside grasp function)
    // ***
    if ( !grasp(goal) )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::hafGrasp] Call to grasp action was unsuccessful" );
        return false;
    }

    // Successfully grasped object!
    ROS_INFO ( "[SquirrelObjectManipulationServer::hafGrasp] Success!" );
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::hafPick ( const squirrel_manipulation_msgs::ManipulationGoalConstPtr &goal )
{
    ROS_INFO ( "[SquirrelObjectManipulationServer::hafPick] Started" );

    // ***
    // Pick up (haf calculation taken care of inside pick function)
    // ***
    if ( !pick(goal) )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::hafPick] Call to pick up action was unsuccessful" );
        return false;
    }

    // Successfully picked object!
    ROS_INFO ( "[SquirrelObjectManipulationServer::hafPick] Success!" );
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::hafGraspFull ( const squirrel_manipulation_msgs::ManipulationGoalConstPtr &goal )
{
    ROS_INFO ( "[SquirrelObjectManipulationServer::hafGraspFull] Started" );

    // ***
    // Prepare for the haf calculation by moving the base so that the object is in view
    // ***
    ROS_INFO ( "[SquirrelObjectManipulationServer::hafGraspFull] Preparing for Haf calculation" );
    if ( !prepareForHafGrasping(goal, HAF_MIN_DIST_) )
        ROS_WARN ( "[SquirrelObjectManipulationServer::hafGraspFull] Failed to prepare for Haf calculation, continuing..." );

    // ***
    // Grasp (haf calculation taken care of inside grasp function)
    // ***
    if ( !grasp(goal) )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::hafGraspFull] Call to grasp action was unsuccessful" );
        return false;
    }

    // Successfully grasped object!
    ROS_INFO ( "[SquirrelObjectManipulationServer::hafGraspFull] Success!" );
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::hafPickFull ( const squirrel_manipulation_msgs::ManipulationGoalConstPtr &goal )
{
    ROS_INFO ( "[SquirrelObjectManipulationServer::hafPickFull] Started" );

    // ***
    // Prepare for the haf calculation by moving the base so that the object is in view
    // ***
    ROS_INFO ( "[SquirrelObjectManipulationServer::hafPickFull] Preparing for Haf calculation" );
    if ( !prepareForHafGrasping(goal, HAF_MIN_DIST_) )
        ROS_WARN ( "[SquirrelObjectManipulationServer::hafPickFull] Failed to prepare for Haf calculation, continuing..." );

    // ***
    // Pick up (haf calculation taken care of inside pick function)
    // ***
    if ( !pick(goal) )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::hafPickFull] Call to pick up action was unsuccessful" );
        return false;
    }

    // Successfully picked object!
    ROS_INFO ( "[SquirrelObjectManipulationServer::hafPickFull] Success!" );
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::moveArmCartesian ( const double &x, const double &y, const double &z,
                                                          const double &xx, const double &yy, const double &zz, const double ww,
                                                          const std::string &message, const float &tolerance )
{
    // Plan end effector to goal position
    feedback_.current_phase = message;
    publishFeedback ( "started" );

    // transfrom goal position from TCP (= hand_palm_link) to wrist (= hand_wrist_link)
    tf::Transform wrist_transform;
    wrist_transform.mult ( tf::Transform(tf::Quaternion(xx, yy, zz, ww), tf::Vector3(x, y, z)), hand_to_wrist_transform_.inverse() );
    geometry_msgs::Pose tcp_pose;
    tcp_pose.position.x = x;
    tcp_pose.position.y = y;
    tcp_pose.position.z = z;
    tcp_pose.orientation.x = xx;
    tcp_pose.orientation.y = yy;
    tcp_pose.orientation.z = zz;
    tcp_pose.orientation.w = ww;
    geometry_msgs::Pose wrist_pose;
    wrist_pose.position.x = wrist_transform.getOrigin().getX();
    wrist_pose.position.y = wrist_transform.getOrigin().getY();
    wrist_pose.position.z = wrist_transform.getOrigin().getZ();
    wrist_pose.orientation.x = wrist_transform.getRotation().getX();
    wrist_pose.orientation.y = wrist_transform.getRotation().getY();
    wrist_pose.orientation.z = wrist_transform.getRotation().getZ();
    wrist_pose.orientation.w = wrist_transform.getRotation().getW();
    //publishGoalMarker ( tcp_pose, 0);
    //publishGoalMarker ( wrist_pose, 1);

    ROS_INFO_STREAM ( "[SquirrelObjectManipulationServer::moveArmCartesian]\n tcp: " << tcp_pose <<
                      "\n wrist: " << wrist_pose);

    /*tx = wrist_transform.getOrigin().getX();
    ty = wrist_transform.getOrigin().getY();
    tz = wrist_transform.getOrigin().getZ();
    mat = tf::Matrix3x3 ( tf::Quaternion(wrist_transform.getRotation().getX(), wrist_transform.getRotation().getY(),
                                         wrist_transform.getRotation().getZ(), wrist_transform.getRotation().getW()) );
    mat.getEulerYPR ( yaw, pitch, roll );
    // --*/

    tf::Matrix3x3 mat ( wrist_transform.getRotation() );
    double roll, pitch, yaw;
    mat.getEulerYPR ( yaw, pitch, roll );

    // Set the values in the message for the motion planner
    // [x y z roll pitch yaw]
    end_eff_goal_.request.positions.resize ( 6 );
    end_eff_goal_.request.positions[0] = wrist_pose.position.x;
    end_eff_goal_.request.positions[1] = wrist_pose.position.y;
    end_eff_goal_.request.positions[2] = wrist_pose.position.z;
    end_eff_goal_.request.positions[3] = roll;
    end_eff_goal_.request.positions[4] = pitch;
    end_eff_goal_.request.positions[5] = yaw;
    end_eff_goal_.request.frame_id = MAP_FRAME_;
    end_eff_goal_.request.max_planning_time = planning_time_;
    end_eff_goal_.request.check_octomap_collision = plan_with_octomap_collisions_;
    end_eff_goal_.request.check_octomap_collision = false;
    end_eff_goal_.request.check_self_collision = plan_with_self_collisions_;
    end_eff_goal_.request.check_self_collision = false;
    end_eff_goal_.request.fold_arm = false;
    end_eff_goal_.request.min_distance_before_folding = 0.0;
    end_eff_goal_.request.disabled_octomap_link_collision.clear();
    if ( plan_with_octomap_collisions_ && (message.compare(STR_GRASP_) == 0 ||
         message.compare(STR_PICK_) == 0 || message.compare(STR_RETRACT_) == 0 ||
         message.compare(STR_PLACE_) == 0) ) // || message.compare(STR_APPROACH_) == 0) )
    {
        ROS_WARN ( "[SquirrelObjectManipulationServer::moveArmCartesian] Disabling finger joints from collision check" );
        end_eff_goal_.request.disabled_octomap_link_collision.push_back("hand_wrist_link");
        end_eff_goal_.request.disabled_octomap_link_collision.push_back("hand_palm_link");
        end_eff_goal_.request.disabled_octomap_link_collision.push_back("hand_base_link");
        end_eff_goal_.request.disabled_octomap_link_collision.push_back("hand_middle_finger_lower_link");
        end_eff_goal_.request.disabled_octomap_link_collision.push_back("hand_middle_finger_upper_link");
        end_eff_goal_.request.disabled_octomap_link_collision.push_back("hand_left_crank");
        end_eff_goal_.request.disabled_octomap_link_collision.push_back("hand_left_coupler");
        end_eff_goal_.request.disabled_octomap_link_collision.push_back("hand_left_finger_lower_link");
        end_eff_goal_.request.disabled_octomap_link_collision.push_back("hand_left_finger_upper_link");
        end_eff_goal_.request.disabled_octomap_link_collision.push_back("hand_right_crank");
        end_eff_goal_.request.disabled_octomap_link_collision.push_back("hand_right_coupler");
        end_eff_goal_.request.disabled_octomap_link_collision.push_back("hand_right_finger_lower_link");
        end_eff_goal_.request.disabled_octomap_link_collision.push_back("hand_right_finger_upper_link");
        end_eff_goal_.request.disabled_octomap_link_collision.push_back("hand_cableCanal_link");
    }

    publishFeedback ( "planning" );
    // Call the service
    if ( !arm_end_eff_planner_client_->call(end_eff_goal_) )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::moveArmCartesian] Failed to find a plan to '%s' pose [%.2f %.2f %.2f %.2f %.2f %.2f]",
        message.c_str(), x, y, z, roll, pitch, yaw );
        publishFeedback ( "failed" );
        return false;
    }
    // Copy the trajectory
    latest_trajectory_ = end_eff_goal_.response.trajectory;
    // If not a successful result
    if ( end_eff_goal_.response.result != 0 )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::moveArmCartesian] Planning to '%s' pose returned with result %i",
        message.c_str(), end_eff_goal_.response.result );
        publishFeedback ( "failed" );
        return false;
    }
    ROS_INFO ( "[SquirrelObjectManipulationServer::moveArmCartesian] Planning to '%s' pose returned with 0", message.c_str() );

    // Check the trajectory does not do an unnecessary spin
    bool decouple_base_arm = false;
    if ( message.compare(STR_APPROACH_) != 0 )
      decouple_base_arm = checkTrajectoryHasSpin();

    // Send the command to the arm controller
    ROS_INFO ( "[SquirrelObjectManipulationServer::moveArmCartesian] Moving to '%s' pose", message.c_str() );
    publishFeedback ( "commanding" );
    if ( !sendCommandTrajectory(message, decouple_base_arm) )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::moveArmCartesian] Failed to send command to '%s' pose",
        message.c_str() );
        publishFeedback ( "failed" );
        return false;
    }

    if ( tolerance > 0 )
    {
        // Print the goal
        ROS_INFO ( "[SquirrelObjectManipulationServer::moveArmCartesian] Goal: %.2f %.2f %.2f", x, y, z );
        // Get the current pose of the wrist link
        geometry_msgs::PoseStamped link_pose, link_pose_map;
        link_pose.pose.orientation.w = 1.0;
        transformPose ( PLANNING_LINK_, MAP_FRAME_, link_pose, link_pose_map );
        // Get the errors
        float err_x = std::fabs ( link_pose_map.pose.position.x - x );
        float err_y = std::fabs ( link_pose_map.pose.position.y - y );
        float err_z = std::fabs ( link_pose_map.pose.position.z - z );
        int count = 0;
        int max_replan = MAX_REPLAN_TRY_;
        if ( message.compare(STR_GRASP_) == 0 || message.compare(STR_PICK_) == 0 ||
             action_type_ == SquirrelObjectManipulationServer::PLACE )
        {
            max_replan = 1;
        }
        ros::Publisher cmd_pub = n_->advertise<geometry_msgs::Twist> ( "/cmd_vel", 3, this );
        geometry_msgs::Twist base_cmd;
        base_cmd.angular.z = -0.7;
        while ( err_x > tolerance || err_y > tolerance || err_z > tolerance )
        {
            publishFeedback ( "retrying" );
            ROS_WARN ( "[SquirrelObjectManipulationServer::moveArmCartesian] Actual: %.2f %.2f %.2f",
                       link_pose_map.pose.position.x, link_pose_map.pose.position.y, link_pose_map.pose.position.z );
            ROS_WARN ( "[SquirrelObjectManipulationServer::moveArmCartesian] Error: %.2f %.2f %.2f", 
                       err_x, err_y, err_z );

            //std::cout << "Press y to replan or anything else to command the base" << std::endl;
            char keyboard_input = 'y';
            //if ( message.compare(STR_APPROACH_) == 0 )
            //    std::cin >> keyboard_input;
            if ( keyboard_input == 'y' || keyboard_input == 'Y' )
            {
                // Call the service
                if ( !arm_end_eff_planner_client_->call(end_eff_goal_) )
                {
                    ROS_ERROR ( "[SquirrelObjectManipulationServer::moveArmCartesian] Failed to find a plan to '%s' pose [%.2f %.2f %.2f %.2f %.2f %.2f]",
                    message.c_str(), x, y, z, roll, pitch, yaw );
                    publishFeedback ( "failed" );
                    break;
                }
                // Copy the trajectory
                latest_trajectory_ = end_eff_goal_.response.trajectory;
                // If not a successful result
                if ( end_eff_goal_.response.result != 0 )
                {
                    ROS_ERROR ( "[SquirrelObjectManipulationServer::moveArmCartesian] Planning to '%s' pose returned with result %i",
                    message.c_str(), end_eff_goal_.response.result );
                    publishFeedback ( "failed" );
                    break;
                }

                //if ( message.compare(STR_APPROACH_) != 0 )
                //  decouple_base_arm = checkTrajectoryHasSpin();

                // Always true when adjusting
                decouple_base_arm = true;
            
                // Send the command
                if ( !sendCommandTrajectory(message, decouple_base_arm) )
                {
                    ROS_ERROR ( "[SquirrelObjectManipulationServer::moveArmCartesian] Failed to send command to '%s' pose",
                                message.c_str() );
                    publishFeedback ( "failed" );
                    //end_eff_goal_.request.min_distance_before_folding = 0.0;
                    return false;
                }
                // Compute the errors
                transformPose ( PLANNING_LINK_, MAP_FRAME_, link_pose, link_pose_map );
            }
            else
            {
                cmd_pub.publish ( base_cmd );
                ros::Duration(1.5).sleep();
                cmd_pub.publish ( base_cmd );
                ros::Duration(1.5).sleep();
                base_cmd.angular.z = 0;
                cmd_pub.publish ( base_cmd );
                ros::Duration(1.0).sleep();
                //count = max_replan;
            }
            err_x = std::fabs ( link_pose_map.pose.position.x - x );
            err_y = std::fabs ( link_pose_map.pose.position.y - y );
            err_z = std::fabs ( link_pose_map.pose.position.z - z );
            ++count;
            if ( count >= max_replan )
                break;
        }

        if ( count >= max_replan )
            ROS_WARN ( "[SquirrelObjectManipulationServer::moveArmCartesian] Failed to meet tolerance after %i attempts", max_replan );
        else
            ROS_INFO ( "[SquirrelObjectManipulationServer::moveArmCartesian] Successful after %i attempts", max_replan );
    }

    // Successfully moved the arm
   publishFeedback ( "success" );
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::moveArmCartesian ( const geometry_msgs::PoseStamped &goal, const std::string &message, const float &tolerance )
{
    return moveArmCartesian ( goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
                              goal.pose.orientation.x, goal.pose.orientation.y,
                              goal.pose.orientation.z, goal.pose.orientation.w,
                              message, tolerance );
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::moveArmJoints ( const std::vector<double> &joint_values, const std::string &message )
{
    // Plan to specific arm and base joint values
    feedback_.current_phase = message;
    publishFeedback ( "started" );
    if ( joint_values.size() != 8 )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::moveArmJoints] Input joint values has %lu elements, expecting 8",
                    joint_values.size() );
        publishFeedback ( "failed" );
        return false;
    }

    // Set the joint values in the message to the motion planner
    pose_goal_.request.frame_id = JOINT_FRAME_;
    pose_goal_.request.joints.resize ( NUM_BASE_JOINTS_ + NUM_ARM_JOINTS_ );  // [basex basey basez arm_joint1 arm_joint2 arm_joint3 arm_joint4 arm_joint5]
    pose_goal_.request.joints = joint_values;
    pose_goal_.request.max_planning_time = planning_time_;
    pose_goal_.request.check_octomap_collision = plan_with_octomap_collisions_;
    pose_goal_.request.check_self_collision = plan_with_self_collisions_;
    pose_goal_.request.check_octomap_collision = false;
    pose_goal_.request.check_self_collision = false;
    pose_goal_.request.fold_arm = false;
    pose_goal_.request.min_distance_before_folding = 0.0;
   publishFeedback ( "planning" );
    // Call the service
    if ( !arm_pose_planner_client_->call(pose_goal_) )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::moveArmJoints] Failed to find plan arm to '%s' joints [%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f]",
                    message.c_str(), joint_values[0], joint_values[1], joint_values[2], joint_values[3],
                    joint_values[4], joint_values[5], joint_values[6], joint_values[7] );
        publishFeedback ( "failed" );
        return false;
    }
    // Copy the trajectory
    latest_trajectory_ = pose_goal_.response.trajectory;
    // If not a successful result
    if ( pose_goal_.response.result != 0 )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::moveArmJoints] Planning to '%s' joints returned with result %i",
                    message.c_str(), pose_goal_.response.result );
        publishFeedback ( "failed" );
        return false;
    }
    ROS_INFO ( "[SquirrelObjectManipulationServer::moveArmJoints] Planning to '%s' joints returned with 0",
               message.c_str() );

    // Send the command to the arm controller
    ROS_INFO ( "[SquirrelObjectManipulationServer::moveArmJoints] Moving to '%s' joints", message.c_str() );
    publishFeedback ( "commanding" );
    if ( !sendCommandTrajectory(message) )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::moveArmJoints] Failed to send command to '%s' joints",
                    message.c_str() );
        publishFeedback ( "failed" );
        return false;
    }

    // Successfully moved the arm
    publishFeedback ( "success" );
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::sendCommandTrajectory ( const std::string &message,
                                                               const bool &decouple_base_arm )
{
    // Send the command to the arm controller
    publishFeedback ( "commanding" );

    // If execting base and arm together
    if ( !decouple_base_arm )
    {
//        if ( !arm_send_trajectory_client_->call(cmd_goal_) )
//        {
//            ROS_ERROR ( "[SquirrelObjectManipulationServer::sendCommandTrajectory] Failed to send '%s' command",
//                        message.c_str() );
//                        publishFeedback ( "failed" );
//            return false;
//        }

//        // If not a successful result
//        if ( cmd_goal_.response.result != 0 )
//        {
//            ROS_ERROR ( "[SquirrelObjectManipulationServer::sendCommandTrajectory] Sending command to move to '%s' returned with result %i",
//                        message.c_str(), cmd_goal_.response.result );
//            publishFeedback ( "failed" );
//            return false;
//        }

        trajectory_controller_pub_.publish ( latest_trajectory_ );

        // Sleep
        ros::Duration(2.0).sleep();
        // Wait for trajectory to finish
        publishFeedback ( "waiting for completion" );
        if ( !waitForTrajectoryCompletion() )
        {
            ROS_ERROR ( "[SquirrelObjectManipulationServer::sendCommandTrajectory] Trajectory to '%s' did not complete in time",
                        message.c_str() );
            publishFeedback ( "failed" );
            return false;
        }
    }
    // Otherwise, decoupling the base and arm
    else
    {
        // -- Send the base command first (send the last base pose of the trajectory)
        trajectory_msgs::JointTrajectory base_msg;
        base_msg.joint_names.resize(8);
        base_msg.joint_names[0] = "base_jointx";
        base_msg.joint_names[1] = "base_jointy";
        base_msg.joint_names[2] = "base_jointz";
        base_msg.joint_names[3] = "arm_joint1";
        base_msg.joint_names[4] = "arm_joint2";
        base_msg.joint_names[5] = "arm_joint3";
        base_msg.joint_names[6] = "arm_joint4";
        base_msg.joint_names[7] = "arm_joint5";
        base_msg.points.resize ( 1 );
        // Set the point as the last point in latest_trajectory
        base_msg.points[0].positions = latest_trajectory_.points.back().positions;
        // Override the joint values for the arm with the current values
        std::vector<double> joints = current_joints_;
        base_msg.points[0].positions[3] = joints[3];
        base_msg.points[0].positions[4] = joints[4];
        base_msg.points[0].positions[5] = joints[5];
        base_msg.points[0].positions[6] = joints[6];
        base_msg.points[0].positions[7] = joints[7];
        base_msg.points[0].time_from_start = latest_trajectory_.points[0].time_from_start;
        // Send the trajectory to the controller
        trajectory_controller_pub_.publish ( base_msg );

        // Sleep
        ros::Duration(2.0).sleep();
        // Wait for trajectory to finish
        publishFeedback ( "waiting for base completion" );
        if ( !waitForTrajectoryCompletion() )
        {
            ROS_ERROR ( "[SquirrelObjectManipulationServer::sendCommandTrajectory] Base trajectory for '%s' did not complete in time",
                        message.c_str() );
            publishFeedback ( "failed" );
            return false;
        }

        // -- Send the arm trajectory (send the full arm trajectory and keep the base stationary)
        trajectory_msgs::JointTrajectory arm_msg = latest_trajectory_;
        // Override the joint values for the base with the current values
        joints = current_joints_;
        for ( size_t i = 0; i < arm_msg.points.size(); ++i )
        {
            arm_msg.points[i].positions[0] = joints[0];
            arm_msg.points[i].positions[1] = joints[1];
            arm_msg.points[i].positions[2] = joints[2];
        }
        // Send the trajectory to the controller
        trajectory_controller_pub_.publish ( arm_msg );

        // Sleep
        ros::Duration(2.0).sleep();
        // Wait for trajectory to finish
        publishFeedback ( "waiting for arm completion" );
        if ( !waitForTrajectoryCompletion() )
        {
            ROS_ERROR ( "[SquirrelObjectManipulationServer::sendCommandTrajectory] Arm trajectory for '%s' did not complete in time",
                        message.c_str() );
            publishFeedback ( "failed" );
            return false;
        }

    }

    // Successfully commanded the arm
    ROS_INFO ( "[SquirrelObjectManipulationServer::sendCommandTrajectory] Sending command to move to '%s' returned with 0",
               message.c_str());
    publishFeedback ( "success" );
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::checkTrajectoryHasSpin ()
{
    // Sum up the base orientations in the trajectory
    // If the sum is more than IS_SPIN_THRESHOLD_ (180 degrees) then this is a spin

    // The trajectory must have more than 2 points
    int size_trajectory = latest_trajectory_.points.size();
    if ( size_trajectory < 2 )
        return false;

    // Get the index of the base rotation joint
    int base_z_index = -1;
    for ( size_t i = 0; i < latest_trajectory_.joint_names.size(); ++i )
    {
        if ( latest_trajectory_.joint_names[i] == "base_jointz" )
        {
            base_z_index = i;
            break;
        }
    }

    // If base orientation is not part of the trajectory
    if ( base_z_index < 0 )
        return false;

    // Sum the orientation values
    float orientation_sum = 0.0;
    //float previous_orientation = latest_trajectory_.points[0].positions[base_z_index], current_orientation = 0;
    for ( int i = 1; i < size_trajectory; ++i )
    {
        // Add the absolute difference between current and previous orientation
        // Be careful of the pi boundary
        orientation_sum += std::fabs ( smallestAngleDifference(latest_trajectory_.points[i-1].positions[base_z_index],
                                                               latest_trajectory_.points[i].positions[base_z_index]) );


        /*
        // Going from negative to positive (0 degree transition)
        if ( previous_orientation < 0 && previous_orientation > -0.5*M_PI && current_orientation >= 0 )
            orientation_sum += ( -previous_orientation + current_orientation );
        // Going from positive to negative (0 degree transition)
        else if ( previous_orientation >= 0 && current_orientation < 0 && current_orientation > -0.5*M_PI )
            orientation_sum += ( previous_orientation - current_orientation );
        // Going from positive to negative (pi/-pi degree transition)
        else if ( previous_orientation >= 0 && previous_orientation > 0.5*M_PI && current_orientation < 0 && current_orientation < -0.5*M_PI )
            orientation_sum += ( (M_PI-previous_orientation) + (M_PI-current_orientation) );
        */

    }

    return true;

    if ( orientation_sum >= SPIN_THRESHOLD_ )
        return true;
    else
        return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float SquirrelObjectManipulationServer::smallestAngleDifference ( const float &from,
                                                                  const float &to ) const
{
    // https://gamedev.stackexchange.com/questions/79303/how-can-i-simplify-this-code-to-compute-the-shortest-rotation-between-two-angles
    if ( from == to )
        return 0;
    else if ( from > to )
        return ( -M_PI + std::fmod(from - to + M_PI, 2*M_PI) );
    else
        return ( M_PI - std::fmod(to - from + M_PI, 2*M_PI) );
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::waitForTrajectoryCompletion ( const double &timeout ) const
{
    // The error of the joints state must be small
    bool all_joints_in_position = true;
    vector<double> diff;
    ros::Time start_time = ros::Time::now();
    ros::Duration dur ( timeout );
    int count = 1;
    while ( ros::Time::now() - start_time < dur )
    {
        if ( count % 10 == 0 )
            std::cout << "waiting..." << std::endl;
        ++count;
        diff = poseDiff ( current_cmd_, current_joints_ );
        all_joints_in_position = true;
        for ( size_t i = 0; i < diff.size(); ++i )
        {
            // If one joint is above the threshold then arm is not in the pose
            if ( diff[i] > JOINT_IN_POSITION_THRESHOLD_ )
                all_joints_in_position = false;
        }
        // If not joint flagged as out of position, then trajectory is completed
        if ( all_joints_in_position )
        {
            ros::Duration(0.5).sleep();
            ROS_INFO ( "[SquirrelObjectManipulationServer::waitForTrajectoryCompletion] Trajectory completed successfully" );
            return true;
        }
        // Sleep
        ros::Duration(0.1).sleep();
    }

    // If finished while loop without exiting then trajectory did not finish in time
    ROS_WARN ( "[SquirrelObjectManipulationServer::waitForTrajectoryCompletion] Failed to complete trajectory before timeout of %.2f seconds", timeout );
    return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SquirrelObjectManipulationServer::jointsStateCallBack ( const sensor_msgs::JointStateConstPtr &joints )
{
    for ( size_t i = 0; i < joints->name.size(); ++i )
    {
        if ( joints->name[i].compare("base_jointx") == 0 ) current_joints_[0] = joints->position[i];
        else if ( joints->name[i].compare("base_jointy") == 0 ) current_joints_[1] = joints->position[i];
        else if ( joints->name[i].compare("base_jointz") == 0 ) current_joints_[2] = joints->position[i];
        else if ( joints->name[i].compare("arm_joint1") == 0 ) current_joints_[3] = joints->position[i];
        else if ( joints->name[i].compare("arm_joint2") == 0 ) current_joints_[4] = joints->position[i];
        else if ( joints->name[i].compare("arm_joint3") == 0 ) current_joints_[5] = joints->position[i];
        else if ( joints->name[i].compare("arm_joint4") == 0 ) current_joints_[6] = joints->position[i];
        else if ( joints->name[i].compare("arm_joint5") == 0 ) current_joints_[7] = joints->position[i];
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SquirrelObjectManipulationServer::jointsCommandCallBack ( const trajectory_msgs::JointTrajectoryConstPtr &cmd )
{
    // Store the last joint configuration from the command
    int trajectory_length = cmd->points.size();
    for ( size_t i = 0; i < cmd->points[trajectory_length-1].positions.size(); ++i )
    {
        if ( cmd->joint_names[i].compare("base_jointx") == 0 ) current_cmd_[0] = cmd->points[trajectory_length-1].positions[i];
        else if ( cmd->joint_names[i].compare("base_jointy") == 0 ) current_cmd_[1] = cmd->points[trajectory_length-1].positions[i];
        else if ( cmd->joint_names[i].compare("base_jointz") == 0 ) current_cmd_[2] = cmd->points[trajectory_length-1].positions[i];
        else if ( cmd->joint_names[i].compare("arm_joint1") == 0 ) current_cmd_[3] = cmd->points[trajectory_length-1].positions[i];
        else if ( cmd->joint_names[i].compare("arm_joint2") == 0 ) current_cmd_[4] = cmd->points[trajectory_length-1].positions[i];
        else if ( cmd->joint_names[i].compare("arm_joint3") == 0 ) current_cmd_[5] = cmd->points[trajectory_length-1].positions[i];
        else if ( cmd->joint_names[i].compare("arm_joint4") == 0 ) current_cmd_[6] = cmd->points[trajectory_length-1].positions[i];
        else if ( cmd->joint_names[i].compare("arm_joint5") == 0 ) current_cmd_[7] = cmd->points[trajectory_length-1].positions[i];

    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SquirrelObjectManipulationServer::moveHead ( const float &val )
{
    std_msgs::Float64 pan_msg;
    pan_msg.data = val;
    head_pub_.publish ( pan_msg );
    ros::Duration(0.1).sleep();
    head_pub_.publish ( pan_msg );
    ros::Duration(0.1).sleep();
    head_pub_.publish ( pan_msg );
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::transformPose ( const string &origin_frame, const string &target_frame,
                                                       geometry_msgs::PoseStamped &in, geometry_msgs::PoseStamped &out ) const
{
    // Set the frames
    in.header.frame_id = origin_frame;
    out.header.frame_id = target_frame;
    // Transform using the listerner
    ros::Time common_time;
    std::string* error;
    try
    {
        tf_listener_.getLatestCommonTime ( origin_frame, target_frame, common_time, error );
        in.header.stamp = common_time;
        tf_listener_.transformPose ( target_frame, in, out );
    }
    catch ( tf::TransformException ex )
    {
        // Error occured!
        ROS_ERROR ( "[SquirrelObjectManipulationServer::transformPose] Tf listener exception thrown with message '%s'", ex.what() );
        ros::Duration(1.0).sleep();
        return false;
    }
    // Print out successful transformation
    ROS_INFO ( "[SquirrelObjectManipulationServer::transformPose] Transformed from: \n[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
               in.pose.position.x, in.pose.position.y, in.pose.position.z, in.pose.orientation.x, in.pose.orientation.y, in.pose.orientation.z, in.pose.orientation.w,
               out.pose.position.x, out.pose.position.y, out.pose.position.z, out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z, out.pose.orientation.w );
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::transformPointCloud ( const string &origin_frame, const string &target_frame,
                                                             sensor_msgs::PointCloud2 &in, sensor_msgs::PointCloud2 &out ) const
{
    // Set the frames
    in.header.frame_id = origin_frame;
    out.header.frame_id = target_frame;
    // Transform using the listerner
    ros::Time common_time;
    std::string* error;
    try
    {
        tf_listener_.getLatestCommonTime ( origin_frame, target_frame, common_time, error );
        in.header.stamp = common_time;
        tf::StampedTransform tf;
        tf_listener_.lookupTransform ( origin_frame, target_frame, common_time, tf );
        pcl_ros::transformPointCloud ( target_frame, tf.inverse(), in, out );
    }
    catch ( tf::TransformException ex )
    {
        // Error occured!
        ROS_ERROR ( "[SquirrelObjectManipulationServer::transformPointCloud] Tf listener exception thrown with message '%s'", ex.what() );
        ros::Duration(1.0).sleep();
        return false;
    }
    return true;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::getGripperAndApproachPose ( const squirrel_manipulation_msgs::ManipulationGoalConstPtr &goal,
                                                                   geometry_msgs::PoseStamped &gripper_pose,
                                                                   geometry_msgs::PoseStamped &approach_pose )
{
    // If this is haf grasping/picking
    if ( action_type_ == SquirrelObjectManipulationServer::HAF_GRASP ||
         action_type_ == SquirrelObjectManipulationServer::HAF_PICK ||
         action_type_ == SquirrelObjectManipulationServer::HAF_GRASP_FULL ||
         action_type_ == SquirrelObjectManipulationServer::HAF_PICK_FULL )
    {
        if ( !callHafGrasping(goal, gripper_pose) )
        {
            // Try a second time
            ROS_WARN ( "[SquirrelGraspServer::getGripperAndApproachPose] Call to haf grasping was unsuccessful, trying again..." );
            if ( !callHafGrasping(goal, gripper_pose) )
            {
                ROS_ERROR ( "[SquirrelGraspServer::getGripperAndApproachPose] Call to haf grasping was unsuccessful" );
                return false;
            }
        }
    }
    // Otherwise, use the information in the goal
    else
    {
        // Check the object orientation is valid
        if ( goal->pose.pose.orientation.x == 0 && goal->pose.pose.orientation.y == 0 &&
             goal->pose.pose.orientation.z == 0 && goal->pose.pose.orientation.w == 0 )
        {
            ROS_ERROR ( "[SquirrelGraspServer::getGripperAndApproachPose] Orientation quaternion is invalid" );
            return false;
        }
        // Transform to map frame
        geometry_msgs::PoseStamped goal_pose = goal->pose;
        geometry_msgs::PoseStamped map_goal;
        map_goal.header.frame_id = MAP_FRAME_;
        transformPose ( goal->pose.header.frame_id, MAP_FRAME_, goal_pose, map_goal );
        gripper_pose = map_goal;
        // TODO: determine a proper gripper orientation related to the orientation of the object
        // For now just face the gripper downwards
        gripper_pose.pose.orientation = getDownwardsGripper();
    }

    // Compute gripper height (add half the height of the bounding cylinder and the finger clearance)
    //gripper_pose.pose.position.z += ( goal->object_bounding_cylinder.height/2.0 + FINGER_CLEARANCE_ );
    // HACK
    /*if ( hand_type_ == SquirrelObjectManipulationServer::METAHAND )
        gripper_pose.pose.position.z = 1.05*METAHAND_MINIMUM_HEIGHT_;
    else if ( hand_type_ == SquirrelObjectManipulationServer::SOFTHAND )
        gripper_pose.pose.position.z = 1.05*SOFTHAND_MINIMUM_HEIGHT_;*/


    // HACK
    float max_height = 0.32;
    if ( action_type_ == SquirrelObjectManipulationServer::PLACE )
    {
        ROS_WARN ( "[SquirrelGraspServer::getGripperAndApproachPose] Setting placement to higher z value than original %.2f",
                   gripper_pose.pose.position.z );
        gripper_pose.pose.position.z = max_height - 0.08;
        approach_pose.pose.position.z = max_height;
    }

    // Check that the goal is above the minimum height
    if ( !checkGoalHeight(gripper_pose) )
    {
        ROS_WARN ( "[SquirrelGraspServer::getGripperAndApproachPose] Goal height %.2f is invalid",
                   gripper_pose.pose.position.z );
        if ( gripper_pose.pose.position.z < 0.02 )
            return false;
        // Adjust to the minbimum height then
        if ( hand_type_ == SquirrelObjectManipulationServer::METAHAND )
            gripper_pose.pose.position.z = METAHAND_MINIMUM_HEIGHT_;
        else if ( hand_type_ == SquirrelObjectManipulationServer::SOFTHAND )
            gripper_pose.pose.position.z = SOFTHAND_MINIMUM_HEIGHT_;
    }

    // Compute approach pose (approach_height_ above gripper pose in the map frame)
    approach_pose = gripper_pose;
    approach_pose.pose.position.z += approach_height_;
    if ( approach_pose.pose.position.z > max_height )
    {
        ROS_WARN ( "[SquirrelGraspServer::getGripperAndApproachPose] Approach height %.2f is too heigh",
                   approach_pose.pose.position.z );
        approach_pose.pose.position.z = max_height;
        if ( gripper_pose.pose.position.z < approach_pose.pose.position.z )
            gripper_pose.pose.position.z = approach_pose.pose.position.z - 0.02;
    }


    // Publish the goal end effector pose for visualization
    /*
    // [x y z roll pitch yaw]
    vector<double> gripper_positions ( 6 );
    gripper_positions[0] = gripper_pose.pose.position.x;
    gripper_positions[1] = gripper_pose.pose.position.y;
    gripper_positions[2] = gripper_pose.pose.position.z;
    tf::Matrix3x3 mat ( tf::Quaternion(gripper_pose.pose.orientation.x,
                                       gripper_pose.pose.orientation.y,
                                       gripper_pose.pose.orientation.z,
                                       gripper_pose.pose.orientation.w) );
    double roll, pitch, yaw;
    mat.getEulerYPR ( yaw, pitch, roll );
    gripper_positions[3] = roll;
    gripper_positions[4] = pitch;
    gripper_positions[5] = yaw;
    // Publish marker
    //publishGoalMarker ( gripper_positions );
    */
    publishGoalMarker ( gripper_pose.pose );

    // Return successful
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
vector<double> SquirrelObjectManipulationServer::poseDiff ( const vector<double> &pose1, const vector<double> &pose2 ) const
{
    vector<double> diff;
    // Cannot compare if pose vectors are not the same size
    if ( pose1.size() != pose2.size() )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::poseDiff] Cannot compare inputs with different sizes, %lu %lu",
                    pose1.size(), pose2.size() );
        return diff;
    }

    // Take the absolute difference between the elements
    diff.resize ( pose1.size() );
    for ( size_t i = 0; i < diff.size(); ++i )
        diff[i] = fabs ( pose1[i] - pose2[i] );

    return diff;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::callHafGrasping ( const squirrel_manipulation_msgs::ManipulationGoalConstPtr &goal,
                                                         geometry_msgs::PoseStamped &gripper_pose )
{
    ROS_INFO ( "[SquirrelObjectManipulationServer::callHafGrasping] Started" );

    // Wait for the action server to start
    if ( !haf_client_->waitForServer(ros::Duration(10.0)) )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::callHafGrasping] Waiting for haf grasping server failed" );
        return false;
    }

    // Check if the point cloud from the recognition service was successful
    bool use_recognition_cloud = true;
    if ( recognition_cloud_.data.size() == 0 )
    {
        ROS_WARN ( "[SquirrelObjectManipulationServer::callHafGrasping] Recognition cloud is empty" );
        use_recognition_cloud = false;
    }

    // If using the recognition cloud
    if ( use_recognition_cloud )
    {
        // Transform to map frame
        sensor_msgs::PointCloud2 transformed_recognition_cloud;
        if ( recognition_cloud_.header.frame_id.compare(MAP_FRAME_) == 0 )
            transformed_recognition_cloud = recognition_cloud_;
        else
            transformPointCloud ( recognition_cloud_.header.frame_id, MAP_FRAME_, recognition_cloud_, transformed_recognition_cloud );
        // Convert to pcl
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud ( new pcl::PointCloud<pcl::PointXYZRGB>() );
        pcl::fromROSMsg ( transformed_recognition_cloud, *temp_cloud );
        // Get bounds
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D ( *temp_cloud, min_pt, max_pt );
        std::cout << "Min: " << min_pt[0] << " " << min_pt[1] << ", max: " << max_pt[0] << " " << max_pt[1] << std::endl;
        // Add Haf search area
        min_pt[0] -= (1.25*0.01*HAF_SEARCH_SIZE_);
        min_pt[1] -= (1.25*0.01*HAF_SEARCH_SIZE_);
        max_pt[0] += (1.25*0.01*HAF_SEARCH_SIZE_);
        max_pt[1] += (1.25*0.01*HAF_SEARCH_SIZE_);
        // Add artificial floor
        int num_floor = 100000;
        int current_size = temp_cloud->size();
        temp_cloud->resize ( num_floor + current_size );
        for ( int i = current_size; i < temp_cloud->size(); ++i )
        {
            // Get random point
            temp_cloud->points[i].x = min_pt[0] + static_cast<float>( rand())  / ( static_cast<float>(RAND_MAX/(max_pt[0]-min_pt[0])) );
            temp_cloud->points[i].y = min_pt[1] + static_cast<float>( rand())  / ( static_cast<float>(RAND_MAX/(max_pt[1]-min_pt[1])) );
            //std::cout << temp_cloud->points[i].x << " " << temp_cloud->points[i].y << std::endl;
            temp_cloud->points[i].z = 0.0;
            // Color
            temp_cloud->points[i].r = 100;
            temp_cloud->points[i].g = 100;
            temp_cloud->points[i].b = 100;
        }
        // Convert to ros message
        pcl::toROSMsg ( *temp_cloud, haf_goal_.graspinput.input_pc );
        std::cout << "Frame of haf point cloud: " << haf_goal_.graspinput.input_pc.header.frame_id << std::endl;
    }
    else
    {
        haf_goal_.graspinput.input_pc = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/kinect/depth_registered/points", *n_, ros::Duration(3.0)));
    }

    // Construct the haf grasping goal
    haf_goal_.graspinput.max_calculation_time = ros::Duration ( 30.0 );

    // Set the search parameters according to the classification result
    squirrel_object_perception_msgs::SceneObject scene_object;
    if ( getSceneObject(goal->object_id, scene_object) )
    {
        haf_goal_.graspinput.goal_frame_id = scene_object.header.frame_id;
        if ( haf_goal_.graspinput.goal_frame_id.empty() )
        {
            ROS_WARN ( "[SquirrelObjectManipulationServer::callHafGrasping] Message strore object has no frame id specified, reverting to map" );
            haf_goal_.graspinput.goal_frame_id = MAP_FRAME_;
        }
        haf_goal_.graspinput.grasp_area_center = scene_object.pose.position;
        //haf_goal_.graspinput.grasp_area_length_x = scene_object.bounding_cylinder.diameter;
        //haf_goal_.graspinput.grasp_area_length_y = scene_object.bounding_cylinder.diameter;
        haf_goal_.graspinput.grasp_area_length_x = HAF_SEARCH_SIZE_;
        haf_goal_.graspinput.grasp_area_length_y = HAF_SEARCH_SIZE_;
    }
    // If could not find the object in the database then use the information in the pose field
    else
    {
        haf_goal_.graspinput.goal_frame_id = goal->pose.header.frame_id;
        haf_goal_.graspinput.grasp_area_center = goal->pose.pose.position;
        haf_goal_.graspinput.grasp_area_length_x = HAF_SEARCH_SIZE_;
        haf_goal_.graspinput.grasp_area_length_y = HAF_SEARCH_SIZE_;
    }

    haf_goal_.graspinput.grasp_area_center.z = 0.0;
    haf_goal_.graspinput.approach_vector.x = 0.0;
    haf_goal_.graspinput.approach_vector.y = 0.0;
    haf_goal_.graspinput.approach_vector.z = 1.0;
    haf_goal_.graspinput.gripper_opening_width = 1.0;
    std::cout << "Haf grasp centre:\n" << haf_goal_.graspinput.grasp_area_center << std::endl;

    // Send the goal and wait for the action server to return
    haf_client_->sendGoal ( haf_goal_ );
    bool finished_before_timeout = haf_client_->waitForResult ( ros::Duration(60.0) );
    int count = 0;
    while ( count < 2 && !finished_before_timeout )
    {
        ++count;
        ROS_WARN("[SquirrelObjectManipulationServer::callHafGrasping] Calling Haf client, count = %i", count );
        haf_client_->sendGoal ( haf_goal_ );
        finished_before_timeout = haf_client_->waitForResult ( ros::Duration(60.0) );
    }

    // If the action returned before the timeout, return the grasp pose
    if ( finished_before_timeout )
    {
        if ( haf_client_->getState() ==  actionlib::SimpleClientGoalState::SUCCEEDED )
        {
            //haf_grasping::CalcGraspPointsServerResultConstPtr result = haf_client_->getResult();
            boost::shared_ptr<const haf_grasping::CalcGraspPointsServerResult_<std::allocator<void> > > result = haf_client_->getResult();
            ROS_INFO_STREAM ( "[SquirrelObjectManipulationServer::callHafGrasping] Result: " << result->graspOutput );
            if ( result->graspOutput.eval < -10 )
            {
                ROS_ERROR ( "[SquirrelObjectManipulationServer::callHafGrasping] Haf returned with value %i",
                            result->graspOutput.eval );
                return false;
            }
            else if ( result->graspOutput.eval < 0 )
            {
                ROS_WARN ( "[SquirrelObjectManipulationServer::callHafGrasping] Haf returned with value %i",
                           result->graspOutput.eval );
            }
            // Get the grasp pose from the calculated haf grasp point and vector
            geometry_msgs::PoseStamped haf_pose;
            haf_pose.header.frame_id = result->graspOutput.header.frame_id;
            haf_pose.pose.position.x = result->graspOutput.averagedGraspPoint.x;
            haf_pose.pose.position.y = result->graspOutput.averagedGraspPoint.y;
            haf_pose.pose.position.z = result->graspOutput.averagedGraspPoint.z;// - 0.01;
            haf_pose.pose.orientation = hafToGripperOrientation ( result->graspOutput );
            // Transform to map frame
            transformPose ( haf_pose.header.frame_id, MAP_FRAME_, haf_pose, gripper_pose );
            //std::cout << "Haf pose\n" << haf_pose << std::endl;
            //std::cout << "Gripper pose\n" << gripper_pose << std::endl;
            // Adjust the height to be the center of the object
            // (this will be readjusted in the grasp/pick call by adding half the height)
            //gripper_pose.pose.position.z /= 2.0;
            // Add height to grasp object within the fingers (not the wrist joint reference)
            //gripper_pose->pose.pose.position.z += FINGER_CLEARANCE_;
            return true;
        }
        else
        {
            ROS_ERROR ( "[SquirrelObjectManipulationServer::callHafGrasping] Haf grasping returned unsuccessfully" );
            return false;
        }
    }
    else
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::callHafGrasping] Action did not finish before the time out" );
        return false;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::prepareForHafGrasping ( const squirrel_manipulation_msgs::ManipulationGoalConstPtr &goal,
                                                               const float &min_dist )
{
    // Get the examine waypoints
    feedback_.current_phase = "examine waypoints";
    publishFeedback ( "started" );
    as_.publishFeedback ( feedback_ );
    if ( !examine_waypoint_client_->waitForExistence(ros::Duration(3.0)) )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::prepareForHafGrasping] Examine waypoints server dor not exist" );
        publishFeedback ( "failed" );
        return false;
    }
    // Call the service
    examine_waypoint_goal_.request.object_pose = goal->pose;
    examine_waypoint_goal_.request.bounding_cylinder = goal->object_bounding_cylinder;
    if ( !examine_waypoint_client_->call(examine_waypoint_goal_) )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::prepareForHafGrasping] Failed to call examine waypoints service" );
        publishFeedback ( "failed" );
        return false;
    }
    // If no poses were returned
    if ( examine_waypoint_goal_.response.poses.size() == 0 )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::prepareForHafGrasping] Examine waypoints service returned 0 poses" );
        publishFeedback ( "failed" );
        return false;
    }

    // Get the current pose of the robot in map frame
    geometry_msgs::PoseStamped odom_pose, map_pose;
    odom_pose.header.frame_id = JOINT_FRAME_;
    odom_pose.pose.position.x = current_joints_[0];
    odom_pose.pose.position.y = current_joints_[1];
    odom_pose.pose.orientation.w = 1.0;
    map_pose.header.frame_id = MAP_FRAME_;
    transformPose ( JOINT_FRAME_, MAP_FRAME_, odom_pose, map_pose );
    
    // Get the nearest waypoint that was returned
    int nearest_pose_index = -1;
    float nearest_dist = 1000, d = 0;
    geometry_msgs::Point curr_pt;
    for ( size_t i = 0; i < examine_waypoint_goal_.response.poses.size(); ++i )
    {
        curr_pt = examine_waypoint_goal_.response.poses[i].pose.pose.position;
        
        // Nearest
        /*d = std::sqrt ( (map_pose.pose.position.x-curr_pt.x)*(map_pose.pose.position.x-curr_pt.x) +
                        (map_pose.pose.position.y-curr_pt.y)*(map_pose.pose.position.y-curr_pt.y) );*/
        // Left most position
        d = examine_waypoint_goal_.response.poses[i].pose.pose.position.x;
        
        if ( d < nearest_dist )
        {
            nearest_dist = d;
            nearest_pose_index = i;
        }
    }

    // If found a good waypoint
    if ( nearest_pose_index < 0 || nearest_pose_index >= examine_waypoint_goal_.response.poses.size() )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::prepareForHafGrasping] Nearest waypoint index %i is invalid", nearest_pose_index );
        publishFeedback ( "failed" );
        return false;
    }
    // Success
    publishFeedback ( "success" );

    // Call move base
    feedback_.current_phase = "move base";
    publishFeedback ( "started" );
    move_base_goal_.target_pose.header = examine_waypoint_goal_.response.poses[nearest_pose_index].header;
    move_base_goal_.target_pose.pose = examine_waypoint_goal_.response.poses[nearest_pose_index].pose.pose;
    /*
    // Calculate a position closer to the object (if min_dist is set to something valid)
    if ( min_dist > 0.0 )
    {
        curr_pt = goal->pose.pose.position;
        d = std::sqrt ( (move_base_goal_.target_pose.pose.position.x-curr_pt.x)*(move_base_goal_.target_pose.pose.position.x-curr_pt.x) +
                        (move_base_goal_.target_pose.pose.position.y-curr_pt.y)*(move_base_goal_.target_pose.pose.position.y-curr_pt.y) );
        cout << "Distance from waypoint is " << d << endl;
        if ( d > min_dist )
        {
            // Get the orientation from the object to the move base goal
            float ang = atan2 ( move_base_goal_.target_pose.pose.position.y - curr_pt.y,
                                move_base_goal_.target_pose.pose.position.x - curr_pt.x );
            // Get the point that is min_dist away from the object
            //float mx = min_dist * cos ( ang ), my = min_dist * sin ( ang );
            move_base_goal_.target_pose.pose.position.x = curr_pt.x + min_dist * cos ( ang );
            move_base_goal_.target_pose.pose.position.y = curr_pt.y + min_dist * sin ( ang );
            cout << "angle: " << ang*180/M_PI << " dx " << min_dist*cos(ang) << " dy " << min_dist*cos(ang) << endl;
            ROS_INFO ( "[SquirrelObjectManipulationServer::prepareForHafGrasping] (%.2f, %.2f) -> (%.2f, %.2f)",
                       examine_waypoint_goal_.response.poses[nearest_pose_index].pose.pose.position.x,
                       examine_waypoint_goal_.response.poses[nearest_pose_index].pose.pose.position.y,
                       move_base_goal_.target_pose.pose.position.x,
                       move_base_goal_.target_pose.pose.position.y );
        }
    }
    */

    move_base_client_->sendGoal ( move_base_goal_ );
    bool finished_before_timeout = move_base_client_->waitForResult ( ros::Duration(60.0) );
    // If the action did not return before the timeout
    if ( !finished_before_timeout )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::prepareForHafGrasping] Move base did not finish before the time out" );
        publishFeedback ( "failed" );
        return false;
    }
    // If the action returned with a state other than success
    if ( move_base_client_->getState() !=  actionlib::SimpleClientGoalState::SUCCEEDED )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::prepareHafGrasping] Move base returned unsuccessfully" );
        publishFeedback ( "failed" );
        return false;
    }

    // TODO: tilt angle???
    
    ros::Duration(1.0).sleep();
    ROS_INFO ( "[SquirrelObjectManipulationServer::prepareHafGrasping] Success" );
    publishFeedback ( "success" );
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
geometry_msgs::Quaternion SquirrelObjectManipulationServer::hafToGripperOrientation ( const haf_grasping::GraspOutput &haf_output ) const
{
    // direction of finger movement = vector between grasp the two points
    tf::Vector3 grasp_dir ( haf_output.graspPoint1.x - haf_output.graspPoint2.x,
                            haf_output.graspPoint1.y - haf_output.graspPoint2.y,
                            haf_output.graspPoint1.z - haf_output.graspPoint2.z );
    grasp_dir.normalize();
    // rotation matrix of the hand_palm_link (= TCP)
    // NOTE: hand_palm_link is defined such that:
    //       x points in the closing direction of the two fingers (treating any hand as two-finger gripper)
    //         in KCL hand case: towards the thumb
    //       z points away from the wrist
    //       y is normal to these
    tf::Matrix3x3 rot;
    tf::Vector3 z ( 0., 0., -1 );
    tf::Vector3 y = z.cross ( grasp_dir );
    rot[0][0] = grasp_dir.getX();
    rot[1][0] = grasp_dir.getY();
    rot[2][0] = grasp_dir.getZ();
    rot[0][1] = y.getX();
    rot[1][1] = y.getY();
    rot[2][1] = y.getZ();
    rot[0][2] = z.getX();
    rot[1][2] = z.getY();
    rot[2][2] = z.getZ();
    // Convert rotation matrix to quaternion and this to a message *sigh*
    tf::Quaternion qrot;
    rot.getRotation ( qrot );
    geometry_msgs::Quaternion result;
    tf::quaternionTFToMsg ( qrot, result );
    return result;

    /*geometry_msgs::PoseStamped downwards_gripper, downwards_gripper_haf_frame;
    downwards_gripper.pose.orientation = getDownwardsGripper();
    // Transform to haf_output frame
    transformPose ( MAP_FRAME_, haf_output.header.frame_id, downwards_gripper, downwards_gripper_haf_frame );
    // Transform to quaternion
    tf::Quaternion quat ( 0.0, 0.0, 0.0, 0.0 );
    tf::quaternionMsgToTF ( downwards_gripper_haf_frame.pose.orientation, quat );

    // Create the rotation according to the roll
    // (TODO: what to do with approach vector in haf_output.approachVector??)
    tf::Matrix3x3 ypr;
    //ypr.setEulerYPR ( 0.0, 0.0, haf_output.roll );
    ypr.setEulerYPR ( atan2(haf_output.graspPoint1.y - haf_output.graspPoint2.y,
                            haf_output.graspPoint1.x - haf_output.graspPoint2.x ),
                      0.0, 0.0 );
    tf::Quaternion rotation;
    ypr.getRotation ( rotation );

    // Apply the rotation to get the new orientation
    tf::Quaternion quat_new = rotation * quat;
    quat_new.normalize();

    // Convert the quaternion to a message and return
    geometry_msgs::Quaternion result;
    tf::quaternionTFToMsg ( quat_new, result );
    return result;*/
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
geometry_msgs::Quaternion SquirrelObjectManipulationServer::getDownwardsGripper () const
{
    // http://quaternions.online/
    // Euler: x = 180, y = 0, z = 0
    // Adjust z (-ve direction to not be exactly straight with axis)

    geometry_msgs::Quaternion downwards_gripper;
    if ( hand_type_ == SquirrelObjectManipulationServer::METAHAND )
    {
        downwards_gripper.x = 1.0;
        downwards_gripper.y = 0.0;
        downwards_gripper.z = 0.0;
        downwards_gripper.w = 0.0;
        /*
        downwards_gripper.x = 0.047;
        downwards_gripper.y = 0.777;
        downwards_gripper.z = -0.038;
        downwards_gripper.w = 0.626;*/
    }
    else if ( hand_type_ == SquirrelObjectManipulationServer::SOFTHAND )
    {
        downwards_gripper.x = 0.5;
        downwards_gripper.y = 0.5;
        downwards_gripper.z = -0.5;
        downwards_gripper.w = 0.5;
    }
    return downwards_gripper;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::getSceneObject ( const std::string &object_id,
                                                        squirrel_object_perception_msgs::SceneObject &scene_object ) const
{
    // If the object id is empty
    if ( object_id.empty() )
    {
        ROS_WARN ( "[SquirrelObjectManipulationServer::getSceneObject] Input object_id is empty" );
        return false;
    }

    // Check if MongoDB is running
    ros::ServiceClient mongodb_client = n_->serviceClient<mongodb_store_msgs::MongoInsertMsg>("/message_store/insert");
    if ( !mongodb_client.waitForExistence(ros::Duration(0.5)) )
    {
        ROS_WARN ( "[SquirrelObjectManipulationServer::getSceneObject] Waiting for mongodb store failed" );
        return false;
    }


    ROS_INFO ( "[SquirrelObjectManipulationServer::getSceneObject] Querying the database for object information" );
    mongodb_store::MessageStoreProxy message_store ( *n_ );
    std::vector<boost::shared_ptr<squirrel_object_perception_msgs::SceneObject> > results;
    message_store.query<squirrel_object_perception_msgs::SceneObject> ( results );
    for ( size_t i = 0; i < results.size(); ++i )
    {
        if ( object_id.compare(results[i]->id) == 0 )
        {
            // Found the object in the message store
            scene_object = *results[i];
            return true;
        }
    }

    // If match is negative then try queryNamed and construct a scene object result
    std::vector<boost::shared_ptr<geometry_msgs::PoseStamped> > named_results;
    std::cout << "Querying for object_id " << object_id << std::endl;
    if ( message_store.queryNamed<geometry_msgs::PoseStamped>(object_id, named_results) && named_results.size() > 0 )
    {
        geometry_msgs::PoseStamped named_pose = *named_results[0];
        scene_object.header = named_pose.header;
        scene_object.category = "unknown";
        scene_object.pose = named_pose.pose;
        scene_object.bounding_cylinder.diameter = 0.33;
        scene_object.bounding_cylinder.height = 0.2;
        return true;
    }

    // If match is negative then did not find the object in the message store
    ROS_WARN ( "[SquirrelObjectManipulationServer::getSceneObject] Did not find object '%s' in the message store",
               object_id.c_str() );
    return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::armIsFolded () const
{
    // Get the pose diff of the current joints and the folded pose
    vector<double> current_arm_joints ( NUM_ARM_JOINTS_ );
    for ( int i = 0; i < NUM_ARM_JOINTS_; ++i )
    current_arm_joints[i] = current_joints_[i+NUM_BASE_JOINTS_];
    vector<double> diff = poseDiff ( current_arm_joints, folded_pose );
    // If returned 0 values then return false
    if ( diff.size() == 0 )
        return false;
    // Otherwise check that all values are below the threshold
    for ( size_t i = 0; i < diff.size(); ++i )
    {
        // If one joint is above the threshold then arm is not in the pose
        if ( diff[i] > JOINT_IN_POSITION_THRESHOLD_ )
            return false;
    }
    // All joints are near
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::armIsUnfolded () const
{
    // Get the pose diff of the current joints and the unfolded pose
    vector<double> current_arm_joints ( NUM_ARM_JOINTS_ );
    for ( int i = 0; i < NUM_ARM_JOINTS_; ++i )
    current_arm_joints[i] = current_joints_[i+NUM_BASE_JOINTS_];
    vector<double> diff = poseDiff ( current_arm_joints, unfolded_pose );
    // If returned 0 values then return false
    if ( diff.size() == 0 )
        return false;
    // Otherwise check that all values are below the threshold
    for ( size_t i = 0; i < diff.size(); ++i )
    {
        // If one joint is above the threshold then arm is not in the pose
        if ( diff[i] > JOINT_IN_POSITION_THRESHOLD_ )
            return false;
    }
    // All joints are near
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::checkGoalHeight ( const geometry_msgs::PoseStamped &goal ) const
{
    // Transform to map frame
    geometry_msgs::PoseStamped in_goal = goal;
    geometry_msgs::PoseStamped map_goal = goal;
    if ( goal.header.frame_id.compare(MAP_FRAME_) != 0 )
    {
        transformPose ( goal.header.frame_id, MAP_FRAME_, in_goal, map_goal );
    }
    // Get the minimum height specific for the hand type
    double min_height = 0.30;
    if ( hand_type_ == SquirrelObjectManipulationServer::METAHAND )
        min_height = METAHAND_MINIMUM_HEIGHT_;
    else if ( hand_type_ == SquirrelObjectManipulationServer::SOFTHAND )
        min_height = SOFTHAND_MINIMUM_HEIGHT_;
    // Check that the goal is above the minimum height
    ROS_WARN ( "[SquirrelObjectManipulationServer::checkGoalHeight] Goal height %.2f", map_goal.pose.position.z );
    if ( map_goal.pose.position.z < min_height )
    {
        ROS_ERROR ( "[SquirrelObjectManipulationServer::checkGoalHeight] Goal height %.2f in frame '%s' is below the minimum threshold %.2f",
                    map_goal.pose.position.z, MAP_FRAME_, min_height );
        return false;
    }

    // Otherwise the goal height is valid
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SquirrelObjectManipulationServer::disableOctomapCollisions ()
{
    plan_with_octomap_collisions_ = false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SquirrelObjectManipulationServer::enableOctomapCollisions ()
{
    plan_with_octomap_collisions_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SquirrelObjectManipulationServer::publishGoalMarker ( const vector<double> &pose, int id )
{
    // Pose is [x y z roll pitch yaw]
    goal_marker_.id = id;
    goal_marker_.type = visualization_msgs::Marker::ARROW;

    // Create two end points
    goal_marker_.points.resize ( 2 );
    // First end points is the grasp point
    goal_marker_.points[0].x = pose[0];
    goal_marker_.points[0].y = pose[1];
    goal_marker_.points[0].z = pose[2];
    // Second end point extends along the direction of the pose
    double length = 0.25;
    double denom = sqrt ( pose[3]*pose[3] + pose[4]*pose[4] + pose[5]*pose[5] );
    goal_marker_.points[1].x = pose[0] + length*pose[3]/denom;
    goal_marker_.points[1].y = pose[1] + length*pose[4]/denom;
    goal_marker_.points[1].z = pose[2] + length*pose[5]/denom;

    // Publish
    goal_pose_pub_.publish ( goal_marker_ );
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SquirrelObjectManipulationServer::publishGoalMarker ( const geometry_msgs::Pose &pose, int id )
{
    // Pose
    float length = 0.3;
    goal_marker_.id = id;
    goal_marker_.type = visualization_msgs::Marker::LINE_LIST;
    goal_marker_.pose = pose;
    goal_marker_.points.resize ( 6 );
    goal_marker_.colors.resize ( 6 );

    goal_marker_.points[0].x = 0;
    goal_marker_.points[0].y = 0;
    goal_marker_.points[0].z = 0;
    goal_marker_.points[1].x = length;
    goal_marker_.points[1].y = 0;
    goal_marker_.points[1].z = 0;
    goal_marker_.colors[0].r = 1.0;
    goal_marker_.colors[0].g = 0.0;
    goal_marker_.colors[0].b = 0.0;
    goal_marker_.colors[0].a = 1.0;
    goal_marker_.colors[1].r = 1.0;
    goal_marker_.colors[1].g = 0.0;
    goal_marker_.colors[1].b = 0.0;
    goal_marker_.colors[1].a = 1.0;

    goal_marker_.points[2].x = 0;
    goal_marker_.points[2].y = 0;
    goal_marker_.points[2].z = 0;
    goal_marker_.points[3].x = 0;
    goal_marker_.points[3].y = length;
    goal_marker_.points[3].z = 0;
    goal_marker_.colors[2].r = 0.0;
    goal_marker_.colors[2].g = 1.0;
    goal_marker_.colors[2].b = 0.0;
    goal_marker_.colors[2].a = 1.0;
    goal_marker_.colors[3].r = 0.0;
    goal_marker_.colors[3].g = 1.0;
    goal_marker_.colors[3].b = 0.0;
    goal_marker_.colors[3].a = 1.0;

    goal_marker_.points[4].x = 0;
    goal_marker_.points[4].y = 0;
    goal_marker_.points[4].z = 0;
    goal_marker_.points[5].x = 0;
    goal_marker_.points[5].y = 0;
    goal_marker_.points[5].z = length;
    goal_marker_.colors[4].r = 0.0;
    goal_marker_.colors[4].g = 0.0;
    goal_marker_.colors[4].b = 1.0;
    goal_marker_.colors[4].a = 1.0;
    goal_marker_.colors[5].r = 0.0;
    goal_marker_.colors[5].g = 0.0;
    goal_marker_.colors[5].b = 1.0;
    goal_marker_.colors[5].a = 1.0;

    // Publish
    goal_pose_pub_.publish ( goal_marker_ );
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SquirrelObjectManipulationServer::publishFeedback ( const std::string &status )
{
    feedback_.current_status = status;
    as_.publishFeedback ( feedback_ );
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
string addNodeName ( const std::string &str )
{
    return "/" + string(NODE_NAME_) + "/" + str;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main ( int argc, char **argv )
{
    ros::init ( argc, argv, NODE_NAME_ );
    ros::NodeHandle n ( "" );

    string action_name = NODE_NAME_;
    if ( !n.getParam(addNodeName("action_name"), action_name) )
    ROS_WARN ( "[SquirrelObjectManipulationServer] No input action name, using default '%s'", action_name.c_str() );

    // Create the server
    SquirrelObjectManipulationServer object_manipulation_server ( n, action_name );
    if ( !object_manipulation_server.initialize() )
    {
        ROS_WARN ( "[SquirrelObjectManipulationServer] Could not initialize" );
        ros::shutdown ;
        return EXIT_FAILURE;
    }
    // Listen to action calls
    ros::spin();

    // Shutdown and exit
    ros::shutdown();
    return EXIT_SUCCESS;
}
