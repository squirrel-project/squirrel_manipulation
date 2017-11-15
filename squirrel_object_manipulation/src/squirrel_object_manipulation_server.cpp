#include "squirrel_object_manipulation/squirrel_object_manipulation_server.h"

using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
SquirrelObjectManipulationServer::SquirrelObjectManipulationServer ( ros::NodeHandle &n, const std::string &action_name ) :
	n_ ( new ros::NodeHandle(n) ),
	action_name_ ( action_name ),
	as_ ( *n_, action_name, boost::bind(&SquirrelObjectManipulationServer::actionServerCallBack, this, _1), false)
{
	as_.start();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
SquirrelObjectManipulationServer::~SquirrelObjectManipulationServer ()
{}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::initialize ( const std::string &hand_name )
{
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
		ROS_ERROR ( "[SquirrelObjectManipulationServer::initialize] Unknown hand name %s ", hand_name.c_str() );
		return false;
	}

	arm_unfold_client_ = new ros::ServiceClient ( n_->serviceClient<squirrel_motion_planner_msgs::UnfoldArm>("/squirrel_8dof_planning/unfold_arm") );
	arm_end_eff_planner_client_ = new ros::ServiceClient ( n_->serviceClient<squirrel_motion_planner_msgs::PlanEndEffector>("/squirrel_8dof_planning/find_plan_end_effector") );
	arm_pose_planner_client_ = new ros::ServiceClient ( n_->serviceClient<squirrel_motion_planner_msgs::PlanPose>("/squirrel_8dof_planning/find_plan_pose") );
	arm_send_trajectory_client_ = new ros::ServiceClient ( n_->serviceClient<squirrel_motion_planner_msgs::SendControlCommand>("/squirrel_8dof_planning/send_trajectory_controller") );
	hand_available_ = false;
	if ( hand_type_ == SquirrelObjectManipulationServer::METAHAND )
	{
		hand_client_ = new ros::ServiceClient ( n_->serviceClient<kclhand_control::HandOperationMode>("/hand_operation_mode") );
		hand_available_ = hand_client_->waitForExistence ( ros::Duration(3.0) );
		if ( !hand_available_ )
			ROS_WARN ( "[SquirrelObjectManipulationServer::initialize] KCL hand operation service unavailable, are you running in simulation?" );
	} else if ( hand_type_ == SquirrelObjectManipulationServer::SOFTHAND )
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

	if ( loaded_poses.size() % 5 == 0 )
	{
		// Folded pose is the first 5 values
		folded_pose.resize ( 5 );
		folded_pose[0] = loaded_poses[0];
		folded_pose[1] = loaded_poses[1];
		folded_pose[2] = loaded_poses[2];
		folded_pose[3] = loaded_poses[3];
		folded_pose[4] = loaded_poses[4];
		// Unfolded pose is the last 5 values
		int num_vals = loaded_poses.size();
		unfolded_pose.resize ( 5 );
		unfolded_pose[0] = loaded_poses[num_vals-5];
		unfolded_pose[1] = loaded_poses[num_vals-4];
		unfolded_pose[2] = loaded_poses[num_vals-3];
		unfolded_pose[3] = loaded_poses[num_vals-2];
		unfolded_pose[4] = loaded_poses[num_vals-1];

		cout << "Unfolded position: " << unfolded_pose[0] << " " << unfolded_pose[1] << " "
			<< unfolded_pose[2] << " " << unfolded_pose[3] << " " << unfolded_pose[4] << endl;
	}
	else
	{
		ROS_ERROR ( "[SquirrelObjectManipulationServer::initialize] Parameter list 'trajectory_folding_arm' is not divisible by 5, folding arm trajectory has not been loaded" );
		return false;
	}

	goal_pose_pub_ = n_->advertise<visualization_msgs::Marker> ( addNodeName("goal_pose"), 3, this );
	goal_marker_.header.frame_id = PLANNING_FRAME_;
	goal_marker_.ns = "grasp_vector";
	goal_marker_.id = 0;
	goal_marker_.type = visualization_msgs::Marker::ARROW;
	goal_marker_.action = visualization_msgs::Marker::ADD;
	goal_marker_.lifetime = ros::Duration();
	goal_marker_.color.r = 1;
	goal_marker_.color.g = 0;
	goal_marker_.color.b = 0;
	goal_marker_.color.a = 1;
	goal_marker_.scale.x = 0.05;  // shaft diameter
	goal_marker_.scale.y = 0.1;  // head diameter
	goal_marker_.scale.z = 0.1;  // head length

	current_joints_.resize ( 8 );  // 5 for arm and 3 for base
	current_cmd_.resize ( 8 );
	joints_state_sub_ = n_->subscribe ( "/joint_states", 1, &SquirrelObjectManipulationServer::jointsStateCallBack, this );
	joints_command_sub_ = n_->subscribe ( "/arm_controller/joint_trajectory_controller/command", 1, &SquirrelObjectManipulationServer::jointsCommandCallBack, this );

	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SquirrelObjectManipulationServer::actionServerCallBack ( const squirrel_manipulation_msgs::BlindGraspGoalConstPtr &goal )
{
	ROS_INFO ( "[SquirrelObjectManipulationServer::actionServerCallBack] Started" );

	// Checking the command (TODO: change the action definition to have action type as input, right now can exploit the obejct_id field)
	action_type_ = SquirrelObjectManipulationServer::UNKNOWN_ACTION;
	feedback_.current_phase = "checking input command";
	feedback_.current_status = "started";
	as_.publishFeedback ( feedback_ );
	if ( goal->object_id.compare("open") == 0 || goal->object_id.compare("open hand") == 0 )
		action_type_ = SquirrelObjectManipulationServer::OPEN_HAND_ACTION;
	else if ( goal->object_id.compare("close") == 0 || goal->object_id.compare("close hand") == 0 )
		action_type_ = SquirrelObjectManipulationServer::CLOSE_HAND_ACTION;
	else if ( goal->object_id.compare("grasp") == 0 || goal->object_id.compare("grasp object") == 0 )
		action_type_ = SquirrelObjectManipulationServer::GRASP;
	else if ( goal->object_id.compare("place") == 0 || goal->object_id.compare("place object") == 0 )
		action_type_ = SquirrelObjectManipulationServer::PLACE;
	if ( action_type_ == SquirrelObjectManipulationServer::UNKNOWN_ACTION )
	{
		ROS_WARN ( "[SquirrelObjectManipulationServer::actionServerCallBack] Could not interpret input command %s", goal->object_id.c_str() );
		// Set the result to something here
		feedback_.current_status = "failed";
		as_.setAborted ( result_ );
		return;
	}
	feedback_.current_status = "success";
	as_.publishFeedback ( feedback_ );

	// Check if the arm is folded
	feedback_.current_phase = "checking arm configuration";
	feedback_.current_status = "started";
	as_.publishFeedback ( feedback_ );
	if ( armIsFolded() )
	{
		ROS_WARN ( "[SquirrelObjectManipulationServer::actionServerCallBack] Arm is folded, it must be unfolded first before manipulating objects" );
		// Set the result to something here
		feedback_.current_status = "failed";
		as_.setAborted ( result_ );
		return;
	}
	feedback_.current_status = "success";
	as_.publishFeedback ( feedback_ );

	// Call the appropriate action
	bool success = false;
	if ( action_type_ == SquirrelObjectManipulationServer::OPEN_HAND_ACTION ||
			action_type_ == SquirrelObjectManipulationServer::CLOSE_HAND_ACTION )
	{
		success = handActuate();
	}
	else if ( action_type_ == SquirrelObjectManipulationServer::GRASP )
	{
		success = grasp ( goal );
	}
	else if ( action_type_ == SquirrelObjectManipulationServer::PLACE )
	{
		success = place ( goal );
	}

	if ( success )
	{
		ROS_INFO ( "[SquirrelObjectManipulationServer::actionServerCallBack] Succeeded!" );
		// Set the action state to succeeded
		as_.setSucceeded ( result_ );
	}
	else
	{
		ROS_ERROR ( "[SquirrelObjectManipulationServer::actionServerCallBack] Failed!" );
		// Set the action state to succeeded
		as_.setAborted ( result_ );
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::handActuate ()
{
	if ( hand_type_ == SquirrelObjectManipulationServer::METAHAND )
	{
		if ( action_type_ == SquirrelObjectManipulationServer::OPEN_HAND_ACTION )
			return metahandActuate ( SquirrelObjectManipulationServer::OPEN_METAHAND );
		else if ( action_type_ == SquirrelObjectManipulationServer::CLOSE_HAND_ACTION )
			return metahandActuate ( SquirrelObjectManipulationServer::CLOSE_METAHAND );
		else
			return false;
	}
	else if ( hand_type_ == SquirrelObjectManipulationServer::SOFTHAND )
	{
		if ( action_type_ == SquirrelObjectManipulationServer::OPEN_HAND_ACTION )
			return softhandActuate ( SquirrelObjectManipulationServer::OPEN_SOFTHAND );
		else if ( action_type_ == SquirrelObjectManipulationServer::CLOSE_HAND_ACTION )
			return softhandActuate ( SquirrelObjectManipulationServer::CLOSE_SOFTHAND );
		else
			return false;

	}
	else
		return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::metahandActuate ( const MetahandActuations &methand_action )
{
	ROS_INFO ( "[SquirrelObjectManipulationServer::metahandActuate] Started" );

	// Set the phase of the action
	if ( methand_action == SquirrelObjectManipulationServer::OPEN_METAHAND )
		feedback_.current_phase = "opening hand";
	else if ( methand_action == SquirrelObjectManipulationServer::CLOSE_METAHAND )
		feedback_.current_phase = "closing hand";
	else
		return false;

	feedback_.current_status = "starting";
	as_.publishFeedback ( feedback_ );

	// Can only actuate the hand if the hand service is available
	if ( hand_available_ )
	{
		// Send the actuation command
		hand_goal_kcl_.request.operation_mode = methand_action;
		feedback_.current_status = "commanding";
		if ( !hand_client_->call(hand_goal_kcl_) )
		{
			ROS_WARN ( "[SquirrelObjectManipulationServer::metahandActuate] Failed to actuate hand" );
			feedback_.current_status = "failed";
			return false;
		}
	}
	else
	{
		ROS_WARN ( "[SquirrelObjectManipulationServer::metahandActuate] Cannot actuate hand because service is unavailable" );
		feedback_.current_status = "failed";
	}

	// Successfully actuated the metahand
	ROS_WARN ( "[SquirrelObjectManipulationServer::metahandActuate] Success!");
	feedback_.current_status = "success";
	as_.publishFeedback ( feedback_ );
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::softhandActuate ( const SofthandActuations &softhand_action)
{
	ROS_INFO ( "[SquirrelObjectManipulationServer::softhandActuate] Started" );

	if ( softhand_action == SquirrelObjectManipulationServer::OPEN_SOFTHAND )
		feedback_.current_phase = "opening hand";
	else if ( softhand_action == SquirrelObjectManipulationServer::CLOSE_SOFTHAND )
		feedback_.current_phase = "closing hand";
	else
		return false;

	feedback_.current_status = "starting";
	as_.publishFeedback ( feedback_ );

	if ( hand_available_ )
	{
		if ( softhand_action == SquirrelObjectManipulationServer::OPEN_SOFTHAND )
		{ 
			hand_goal_softhand_.request.position = 0.0;			
		} else 
		{
			hand_goal_softhand_.request.position = 0.9;			
		}
		feedback_.current_status = "commanding";
		if ( !hand_client_->call(hand_goal_softhand_) )
		{
			ROS_WARN ( "[SquirrelObjectManipulationServer::softhandActuate] Failed to actuate hand" );
			feedback_.current_status = "failed";
			return false;
		}		
	}
	else
	{
		ROS_WARN ( "[SquirrelObjectManipulationServer::softhandActuate] Cannot actuate hand because service is unavailable" );
		feedback_.current_status = "failed";
	}

	// Successfully actuated the metahand
	ROS_WARN ( "[SquirrelObjectManipulationServer::softhandActuate] Success!");
	feedback_.current_status = "success";
	as_.publishFeedback ( feedback_ );
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::grasp ( const squirrel_manipulation_msgs::BlindGraspGoalConstPtr &goal )
{
	if ( hand_type_ == SquirrelObjectManipulationServer::METAHAND )
		return metahandGrasp ( goal );
	else if ( hand_type_ == SquirrelObjectManipulationServer::SOFTHAND )
		return softhandGrasp ( goal );
	else
		return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::metahandGrasp ( const squirrel_manipulation_msgs::BlindGraspGoalConstPtr &goal )
{
	ROS_INFO ( "[SquirrelObjectManipulationServer::metahandGrasp] Started" );

	// Compute approach pose (15 cm above input pose in the map frame)
	string goal_frame = goal->heap_center_pose.header.frame_id;
	geometry_msgs::PoseStamped goal_pose = goal->heap_center_pose;
	// Transform to map frame
	geometry_msgs::PoseStamped map_goal;
	transformPose ( goal_frame, MAP_FRAME_, goal_pose, map_goal );
	// Add height
	map_goal.pose.position.z += APPROACH_HEIGHT_;
	map_goal.header.frame_id = MAP_FRAME_;

	// Transform to planning frame (also map but could be different!)
	geometry_msgs::PoseStamped grasp_goal;
	transformPose ( goal_frame, PLANNING_FRAME_, goal_pose, grasp_goal );
	grasp_goal.header.frame_id = PLANNING_FRAME_;
	geometry_msgs::PoseStamped approach_goal;
	transformPose ( MAP_FRAME_, PLANNING_FRAME_, map_goal, approach_goal );
	approach_goal.header.frame_id = PLANNING_FRAME_;

	// Publish the goal end effector pose
	vector<double> grasp_positions ( 6 ); // [x y z roll pitch yaw]
	grasp_positions[0] = grasp_goal.pose.position.x;
	grasp_positions[1] = grasp_goal.pose.position.y;
	grasp_positions[2] = grasp_goal.pose.position.z;
	tf::Matrix3x3 mat ( tf::Quaternion(grasp_goal.pose.orientation.x,
				grasp_goal.pose.orientation.y,
				grasp_goal.pose.orientation.z,
				grasp_goal.pose.orientation.w) );
	double roll, pitch, yaw;
	mat.getEulerYPR ( yaw, pitch, roll );
	grasp_positions[3] = roll;
	grasp_positions[4] = pitch;
	grasp_positions[5] = yaw;
	// Publish marker
	publishGoalMarker ( grasp_positions );


	// ***
	// Open hand
	// ***
	if ( !metahandActuate(SquirrelObjectManipulationServer::OPEN_METAHAND) )
	{
		ROS_WARN ( "[SquirrelGraspServer::metahandGrasp] Failed to open hand" );
		// return false;
	}

	// ***
	// Approach
	// ***
	if ( !moveArmPTP(approach_goal, "approach") )
	{
		ROS_WARN ( "[SquirrelObjectManipulationServer::metahandGrasp] Failed to reach approach pose");
		return false;
	}

	// ***
	// Grasp
	// ***
	if ( !moveArmPTP(grasp_goal, "grasp") )
	{
		ROS_WARN ( "[SquirrelObjectManipulationServer::metahandGrasp] Failed to reach grasp pose");
		return false;
	}

	// ***
	// Close hand
	// ***
	if ( !metahandActuate(SquirrelObjectManipulationServer::CLOSE_METAHAND) )
	{
		ROS_WARN ( "[SquirrelGraspServer::metahandGrasp] Failed to close hand" );
		// return false;
	}

	// ***
	// Retract to unfolded position for carrying
	// ***
	vector<double> retract_joint_values ( 8 ); // [basex basey basez arm_joint1 arm_joint2 arm_joint3 arm_joint4 arm_joint5]
	retract_joint_values[0] = current_joints_[0];
	retract_joint_values[1] = current_joints_[1];
	retract_joint_values[2] = current_joints_[2];
	retract_joint_values[3] = unfolded_pose[0];
	retract_joint_values[4] = unfolded_pose[1];
	retract_joint_values[5] = unfolded_pose[2];
	retract_joint_values[6] = unfolded_pose[3];
	retract_joint_values[7] = unfolded_pose[4];
	if ( !moveArmJoints(retract_joint_values, "retract") )
	{
		ROS_WARN ( "[SquirrelObjectManipulationServer::metahandGrasp] Failed to reach retract position");
		return false;
	}

	// Successfully grasped object!
	ROS_INFO ( "[SquirrelObjectManipulationServer::metahandGrasp] Success!");
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::softhandGrasp ( const squirrel_manipulation_msgs::BlindGraspGoalConstPtr &goal )
{
	ROS_INFO ( "[SquirrelObjectManipulationServer::softhandGrasp] Started" );

	// Compute approach pose (15 cm above input pose in the map frame)
	string goal_frame = goal->heap_center_pose.header.frame_id;
	geometry_msgs::PoseStamped goal_pose = goal->heap_center_pose;
	// Transform to map frame
	geometry_msgs::PoseStamped map_goal;
	transformPose ( goal_frame, MAP_FRAME_, goal_pose, map_goal );
	// Add height
	map_goal.pose.position.z += APPROACH_HEIGHT_;
	map_goal.header.frame_id = MAP_FRAME_;

	// Transform to planning frame (also map but could be different!)
	geometry_msgs::PoseStamped grasp_goal;
	transformPose ( goal_frame, PLANNING_FRAME_, goal_pose, grasp_goal );
	grasp_goal.header.frame_id = PLANNING_FRAME_;
	geometry_msgs::PoseStamped approach_goal;
	transformPose ( MAP_FRAME_, PLANNING_FRAME_, map_goal, approach_goal );
	approach_goal.header.frame_id = PLANNING_FRAME_;

	// Publish the goal end effector pose
	vector<double> grasp_positions ( 6 ); // [x y z roll pitch yaw]
	grasp_positions[0] = grasp_goal.pose.position.x;
	grasp_positions[1] = grasp_goal.pose.position.y;
	grasp_positions[2] = grasp_goal.pose.position.z;
	tf::Matrix3x3 mat ( tf::Quaternion(grasp_goal.pose.orientation.x,
				grasp_goal.pose.orientation.y,
				grasp_goal.pose.orientation.z,
				grasp_goal.pose.orientation.w) );
	double roll, pitch, yaw;
	mat.getEulerYPR ( yaw, pitch, roll );
	grasp_positions[3] = roll;
	grasp_positions[4] = pitch;
	grasp_positions[5] = yaw;
	// Publish marker
	publishGoalMarker ( grasp_positions );


	// ***
	// Open hand
	// ***
	if ( !softhandActuate(SquirrelObjectManipulationServer::OPEN_SOFTHAND) )
	{
		ROS_WARN ( "[SquirrelGraspServer::softhandGrasp] Failed to open hand" );
		// return false;
	}

	// ***
	// Approach
	// ***
	if ( !moveArmPTP(approach_goal, "approach") )
	{
		ROS_WARN ( "[SquirrelObjectManipulationServer::softhandGrasp] Failed to reach approach pose");
		return false;
	}

	// ***
	// Grasp
	// ***
	if ( !moveArmPTP(grasp_goal, "grasp") )
	{
		ROS_WARN ( "[SquirrelObjectManipulationServer::softhandGrasp] Failed to reach grasp pose");
		return false;
	}

	// ***
	// Close hand
	// ***
	if ( !softhandActuate(SquirrelObjectManipulationServer::CLOSE_SOFTHAND) )
	{
		ROS_WARN ( "[SquirrelGraspServer::softhandGrasp] Failed to close hand" );
		// return false;
	}

	// ***
	// Retract to unfolded position for carrying
	// ***
	vector<double> retract_joint_values ( 8 ); // [basex basey basez arm_joint1 arm_joint2 arm_joint3 arm_joint4 arm_joint5]
	retract_joint_values[0] = current_joints_[0];
	retract_joint_values[1] = current_joints_[1];
	retract_joint_values[2] = current_joints_[2];
	retract_joint_values[3] = unfolded_pose[0];
	retract_joint_values[4] = unfolded_pose[1];
	retract_joint_values[5] = unfolded_pose[2];
	retract_joint_values[6] = unfolded_pose[3];
	retract_joint_values[7] = unfolded_pose[4];
	if ( !moveArmJoints(retract_joint_values, "retract") )
	{
		ROS_WARN ( "[SquirrelObjectManipulationServer::softhandGrasp] Failed to reach retract position");
		return false;
	}

	// Successfully grasped object!
	ROS_INFO ( "[SquirrelObjectManipulationServer::softhandGrasp] Success!");
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::place ( const squirrel_manipulation_msgs::BlindGraspGoalConstPtr &goal )
{
	if ( hand_type_ == SquirrelObjectManipulationServer::METAHAND )
		return metahandPlace ( goal );
	else if ( hand_type_ == SquirrelObjectManipulationServer::SOFTHAND )
		return softhandPlace ( goal );
	else
		return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::metahandPlace ( const squirrel_manipulation_msgs::BlindGraspGoalConstPtr &goal )
{
	ROS_INFO ( "[SquirrelObjectManipulationServer::metahandPlace] Started" );

	// Compute approach pose (15 cm above input pose in the map frame)
	string goal_frame = goal->heap_center_pose.header.frame_id;
	geometry_msgs::PoseStamped goal_pose = goal->heap_center_pose;
	// Transform to map frame
	geometry_msgs::PoseStamped map_goal;
	transformPose ( goal_frame, MAP_FRAME_, goal_pose, map_goal );
	// Add height
	map_goal.pose.position.z += APPROACH_HEIGHT_;
	map_goal.header.frame_id = MAP_FRAME_;

	// Transform to planning frame (also map but could be different!)
	geometry_msgs::PoseStamped grasp_goal;
	transformPose ( goal_frame, PLANNING_FRAME_, goal_pose, grasp_goal );
	grasp_goal.header.frame_id = PLANNING_FRAME_;
	geometry_msgs::PoseStamped approach_goal;
	transformPose ( MAP_FRAME_, PLANNING_FRAME_, map_goal, approach_goal );
	approach_goal.header.frame_id = PLANNING_FRAME_;

	// Publish the goal end effector pose
	vector<double> grasp_positions ( 6 ); // [x y z roll pitch yaw]
	grasp_positions[0] = grasp_goal.pose.position.x;
	grasp_positions[1] = grasp_goal.pose.position.y;
	grasp_positions[2] = grasp_goal.pose.position.z;
	tf::Matrix3x3 mat ( tf::Quaternion(grasp_goal.pose.orientation.x,
				grasp_goal.pose.orientation.y,
				grasp_goal.pose.orientation.z,
				grasp_goal.pose.orientation.w) );
	double roll, pitch, yaw;
	mat.getEulerYPR ( yaw, pitch, roll );
	grasp_positions[3] = roll;
	grasp_positions[4] = pitch;
	grasp_positions[5] = yaw;
	// Publish marker
	publishGoalMarker ( grasp_positions );


	// ***
	// Approach
	// ***
	if ( !moveArmPTP(approach_goal, "approach") )
	{
		ROS_WARN ( "[SquirrelObjectManipulationServer::metahandPlace] Failed to reach approach pose");
		return false;
	}

	// ***
	// Place
	// ***
	if ( !moveArmPTP(grasp_goal, "place") )
	{
		ROS_WARN ( "[SquirrelObjectManipulationServer::metahandPlace] Failed to reach place pose");
		return false;
	}

	// ***
	// Open hand
	// ***
	if ( !metahandActuate(SquirrelObjectManipulationServer::OPEN_METAHAND) )
	{
		ROS_WARN ( "[SquirrelGraspServer::metahandPlace] Failed to open hand" );
		// return false;
	}

	// ***
	// Retract to unfolded position
	// ***
	vector<double> retract_joint_values ( 8 ); // [basex basey basez arm_joint1 arm_joint2 arm_joint3 arm_joint4 arm_joint5]
	retract_joint_values[0] = current_joints_[0];
	retract_joint_values[1] = current_joints_[1];
	retract_joint_values[2] = current_joints_[2];
	retract_joint_values[3] = unfolded_pose[0];
	retract_joint_values[4] = unfolded_pose[1];
	retract_joint_values[5] = unfolded_pose[2];
	retract_joint_values[6] = unfolded_pose[3];
	retract_joint_values[7] = unfolded_pose[4];
	if ( !moveArmJoints(retract_joint_values, "retract") )
	{
		ROS_WARN ( "[SquirrelObjectManipulationServer::metahandPlace] Failed to reach retract position");
		return false;
	}

	// Successfully placed object!
	ROS_INFO ( "[SquirrelObjectManipulationServer::metahandPlace] Success!");
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::softhandPlace ( const squirrel_manipulation_msgs::BlindGraspGoalConstPtr &goal )
{
	ROS_INFO ( "[SquirrelObjectManipulationServer::softhandPlace] Started" );

	// TODO Philipp Zech

	return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::moveArmPTP ( const double &x, const double &y, const double &z,
		const double &roll, const double &pitch, const double &yaw,
		const std::string &message )
{
	// Plan end effector to goal position
	feedback_.current_phase = message;
	feedback_.current_status = "starting";
	as_.publishFeedback ( feedback_ );

	// Set the values in the message for the motion planner
	end_eff_goal_.request.positions.resize ( 6 ); // [x y z roll pitch yaw]
	end_eff_goal_.request.positions[0] = x;
	end_eff_goal_.request.positions[1] = y;
	end_eff_goal_.request.positions[2] = z;
	end_eff_goal_.request.positions[3] = roll;
	end_eff_goal_.request.positions[4] = pitch;
	end_eff_goal_.request.positions[5] = yaw;
	end_eff_goal_.request.max_planning_time = 3.0;
	end_eff_goal_.request.check_octomap_collision = false;
	end_eff_goal_.request.check_self_collision = true;
	end_eff_goal_.request.fold_arm = false;
	end_eff_goal_.request.min_distance_before_folding = 0.0;
	feedback_.current_status = "planning";
	// Call the service
	if ( !arm_end_eff_planner_client_->call(end_eff_goal_) )
	{
		ROS_WARN ( "[SquirrelObjectManipulationServer::moveArmPTP] Failed to find a plan to %s pose [%.2f %.2f %.2f %.2f %.2f %.2f]",
				message.c_str(), x, y, z, roll, pitch, yaw );
		// Set the result to something here
		feedback_.current_status = "failed";
		return false;
	}
	// If not a successful result
	if ( end_eff_goal_.response.result != 0 )
	{
		ROS_WARN ( "[SquirrelObjectManipulationServer::moveArmPTP] Planning to %s pose returned with result %i",
				message.c_str(), end_eff_goal_.response.result  );
		// Set the result to something here
		feedback_.current_status = "failed";
		return false;
	}
	else
	{
		ROS_INFO ( "[SquirrelObjectManipulationServer::moveArmPTP] Planning to %s pose returned with 0", message.c_str() );
	}

	// Send the command to the arm controller
	ROS_INFO ( "[SquirrelObjectManipulationServer::moveArmPTP] Moving to %s pose", message.c_str() );
	feedback_.current_status = "commanding";
	if ( !arm_send_trajectory_client_->call(cmd_goal_) )
	{
		ROS_WARN ( "[SquirrelObjectManipulationServer::moveArmPTP] Failed to send command to %s pose", message.c_str() );
		// Set the result to something here
		feedback_.current_status = "failed";
		return false;
	}
	// Sleep
	ros::Duration(2.0).sleep();
	// Wait for trajectory to finish
	feedback_.current_status = "waiting for completion";
	if ( !waitForTrajectoryCompletion() )
	{
		ROS_WARN ( "[SquirrelObjectManipulationServer::moveArmPTP] Trajectory to %s pose did not complete in time",
				message.c_str() );
		// Set the result to something here
		feedback_.current_status = "failed";
		return false;
	}
	// If not a successful result
	if ( cmd_goal_.response.result != 0 )
	{
		ROS_WARN ( "[SquirrelGraspServer::moveArmPTP] Sending command to move to %s pose returned with result %i",
				message.c_str(), cmd_goal_.response.result );
		// Set the result to something here
		feedback_.current_status = "failed";
		return false;
	}
	else
	{
		ROS_INFO ( "[SquirrelObjectManipulationServer::moveArmPTP] Sending command to move to %s pose returned with 0",
				message.c_str());
	}

	// Successfully moved the arm
	feedback_.current_status = "success";
	as_.publishFeedback ( feedback_ );
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::moveArmPTP ( const double &x, const double &y, const double &z,
		const double &xx, const double &yy, const double &zz, const double ww,
		const std::string &message )
{
	// Convert quaternion to Euler
	tf::Matrix3x3 mat ( tf::Quaternion(xx, yy, zz, ww) );
	double roll, pitch, yaw;
	mat.getEulerYPR ( yaw, pitch, roll );
	return moveArmPTP ( x, y, z, roll, pitch, yaw, message );
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::moveArmPTP ( const geometry_msgs::PoseStamped &goal, const std::string &message )
{
	return moveArmPTP ( goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
			goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w,
			message );
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::moveArmJoints ( const std::vector<double> &joint_values, const std::string &message )
{
	feedback_.current_phase = message;
	feedback_.current_status = "starting";
	as_.publishFeedback ( feedback_ );
	if ( joint_values.size() != 8 )
	{
		ROS_WARN ( "[SquirrelObjectManipulationServer::moveArmJoints] Input joint values has %lu elements, expecting 8",
				joint_values.size() );
		// Set the result to something here
		feedback_.current_status = "failed";
		return false;
	}

	// Set the joint values in the message to the motion planner
	pose_goal_.request.joints.resize ( 8 );  // [basex basey basez arm_joint1 arm_joint2 arm_joint3 arm_joint4 arm_joint5]
	pose_goal_.request.joints = joint_values;
	pose_goal_.request.max_planning_time = 3.0;
	pose_goal_.request.check_octomap_collision = false;
	pose_goal_.request.check_self_collision = true;
	pose_goal_.request.fold_arm = false;
	pose_goal_.request.min_distance_before_folding = 0.0;
	feedback_.current_status = "planning";
	// Call the service
	if ( !arm_pose_planner_client_->call(pose_goal_) )
	{
		ROS_WARN ( "[SquirrelObjectManipulationServer::moveArmJoints] Failed to find plan arm to %s joints [%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f]",
				message.c_str(), joint_values[0], joint_values[1], joint_values[2], joint_values[3],
				joint_values[4], joint_values[5], joint_values[6], joint_values[7] );
		// Set the result to something here
		feedback_.current_status = "failed";
		return false;
	}
	// If not a successful result
	if ( pose_goal_.response.result != 0 )
	{
		ROS_WARN ( "[SquirrelObjectManipulationServer::moveArmJoints] Planning to %s joints returned with result %i",
				message.c_str(), pose_goal_.response.result  );
		// Set the result to something here
		feedback_.current_status = "failed";
		return false;
	}
	else
	{
		ROS_INFO ( "[SquirrelObjectManipulationServer::moveArmJoints] Planning to %s joints returned with 0", message.c_str() );
	}

	// Send the command to the arm controller
	ROS_INFO ( "[SquirrelObjectManipulationServer::moveArmJoints] Moving to %s joints", message.c_str() );
	feedback_.current_status = "commanding";
	if ( !arm_send_trajectory_client_->call(cmd_goal_) )
	{
		ROS_WARN ( "[SquirrelObjectManipulationServer::moveArmJoints] Failed to send command to %s joints", message.c_str() );
		// Set the result to something here
		feedback_.current_status = "failed";
		return false;
	}
	// Sleep
	ros::Duration(2.0).sleep();
	// Wait for trajectory to finish
	feedback_.current_status = "waiting for completion";
	if ( !waitForTrajectoryCompletion() )
	{
		ROS_WARN ( "[SquirrelObjectManipulationServer::moveArmJoints] Trajectory to %s joints did not complete in time",
				message.c_str());
		// Set the result to something here
		feedback_.current_status = "failed";
		return false;
	}
	// If not a successful result
	if ( cmd_goal_.response.result != 0 )
	{
		ROS_WARN ( "[SquirrelObjectManipulationServer::moveArmJoints] Sending command to rmove to %s joints returned with result %i",
				message.c_str(), cmd_goal_.response.result  );
		// Set the result to something here
		feedback_.current_status = "failed";
		return false;
	}
	else
	{
		ROS_INFO ( "[SquirrelObjectManipulationServer::moveArmJoints] Sending command to move to %s joints returned with 0",
				message.c_str());
	}

	// Successfully moved the arm
	feedback_.current_status = "success";
	as_.publishFeedback ( feedback_ );
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SquirrelObjectManipulationServer::jointsStateCallBack ( const sensor_msgs::JointStateConstPtr &joints )
{
	//ROS_INFO ( "[SquirrelObjectManipulationServer::jointsStateCallBack] Message received" );
	for ( size_t i = 0; i < joints->name.size(); ++i )
	{
		if ( joints->name[i].compare("base_jointx") == 0 )
		{
			current_joints_[0] = joints->position[i];
		}
		else if ( joints->name[i].compare("base_jointy") == 0 )
		{
			current_joints_[1] = joints->position[i];
		}
		else if ( joints->name[i].compare("base_jointz") == 0 )
		{
			current_joints_[2] = joints->position[i];
		}
		else if ( joints->name[i].compare("arm_joint1") == 0 )
		{
			current_joints_[3] = joints->position[i];
		}
		else if ( joints->name[i].compare("arm_joint2") == 0 )
		{
			current_joints_[4] = joints->position[i];
		}
		else if ( joints->name[i].compare("arm_joint3") == 0 )
		{
			current_joints_[5] = joints->position[i];
		}
		else if ( joints->name[i].compare("arm_joint4") == 0 )
		{
			current_joints_[6] = joints->position[i];
		}
		else if ( joints->name[i].compare("arm_joint5") == 0 )
		{
			current_joints_[7] = joints->position[i];
		}
	}
	//  ROS_INFO ( "[SquirrelObjectManipulationServer::jointsStateCallBack] %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
	//             current_joints_[0], current_joints_[1], current_joints_[2], current_joints_[3],
	//             current_joints_[4], current_joints_[5], current_joints_[6], current_joints_[7] );
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SquirrelObjectManipulationServer::jointsCommandCallBack ( const trajectory_msgs::JointTrajectoryConstPtr &cmd )
{
	//ROS_INFO ( "[SquirrelObjectManipulationServer::jointsCommandCallBack] Message received" );
	// Store the last joint configuration from the command
	int trajectory_length = cmd->points.size();
	for ( size_t i = 0; i < cmd->points[trajectory_length-1].positions.size(); ++i )
	{
		if ( cmd->joint_names[i].compare("base_jointx") == 0 )
		{
			current_cmd_[0] = cmd->points[trajectory_length-1].positions[i];
		}
		else if ( cmd->joint_names[i].compare("base_jointy") == 0 )
		{
			current_cmd_[1] = cmd->points[trajectory_length-1].positions[i];
		}
		else if ( cmd->joint_names[i].compare("base_jointz") == 0 )
		{
			current_cmd_[2] = cmd->points[trajectory_length-1].positions[i];
		}
		else if ( cmd->joint_names[i].compare("arm_joint1") == 0 )
		{
			current_cmd_[3] = cmd->points[trajectory_length-1].positions[i];
		}
		else if ( cmd->joint_names[i].compare("arm_joint2") == 0 )
		{
			current_cmd_[4] = cmd->points[trajectory_length-1].positions[i];
		}
		else if ( cmd->joint_names[i].compare("arm_joint3") == 0 )
		{
			current_cmd_[5] = cmd->points[trajectory_length-1].positions[i];
		}
		else if ( cmd->joint_names[i].compare("arm_joint4") == 0 )
		{
			current_cmd_[6] = cmd->points[trajectory_length-1].positions[i];
		}
		else if ( cmd->joint_names[i].compare("arm_joint5") == 0 )
		{
			current_cmd_[7] = cmd->points[trajectory_length-1].positions[i];
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::transformPose ( const string &origin_frame, const string &target_frame,
		geometry_msgs::PoseStamped &in, geometry_msgs::PoseStamped &out ) const
{
	// Set the frames
	in.header.frame_id = origin_frame;
	out.header.frame_id = target_frame;
	// Transform using the listerner
	try
	{
		tf_listener_.transformPose ( target_frame, in, out );
	}
	catch ( tf::TransformException ex )
	{
		// Error occured!
		ROS_ERROR ( "[SquirrelObjectManipulationServer::transformPose] Tf listener exception thrown with message '%s'",ex.what() );
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
vector<double> SquirrelObjectManipulationServer::poseDiff ( const vector<double> &pose1, const vector<double> &pose2 ) const
{
	vector<double> diff;
	// Cannot compare if pose vectors are not the same size
	if ( pose1.size() != pose2.size() )
	{
		ROS_ERROR ( "[SquirrelObjectManipulationServer::poseDiff] Cannot compare inputs with different sizes, %lu %lu", pose1.size(), pose2.size() );
		return diff;
	}

	// Take the absolute difference between the elements
	diff.resize ( pose1.size() );
	for ( size_t i = 0; i < diff.size(); ++i )
		diff[i] = fabs ( pose1[i] - pose2[i] );

	return diff;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelObjectManipulationServer::armIsFolded () const
{
	// Get the pose diff of the current joints and the folded pose
	vector<double> current_arm_joints ( 5 );
	current_arm_joints[0] = current_joints_[3];
	current_arm_joints[1] = current_joints_[4];
	current_arm_joints[2] = current_joints_[5];
	current_arm_joints[3] = current_joints_[6];
	current_arm_joints[4] = current_joints_[7];
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
	vector<double> current_arm_joints ( 5 );
	current_arm_joints[0] = current_joints_[3];
	current_arm_joints[1] = current_joints_[4];
	current_arm_joints[2] = current_joints_[5];
	current_arm_joints[3] = current_joints_[6];
	current_arm_joints[4] = current_joints_[7];
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
bool SquirrelObjectManipulationServer::waitForTrajectoryCompletion ( const double &timeout )
{
	// The error of the joints state must be small
	bool all_joints_in_position = true;
	vector<double> diff;
	ros::Time start_time = ros::Time::now();
	ros::Duration dur ( timeout );
	while ( ros::Time::now() - start_time < dur )
	{
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
			return true;
	}

	// If finished while loop without exiting then trajectory did not finish in time
	return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SquirrelObjectManipulationServer::publishGoalMarker ( const vector<double> &pose )
{
	// Pose is [x y z roll pitch yaw]

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
		ROS_WARN ( "[SquirrelGraspServer] No input action name, using default %s", action_name.c_str() );
	string hand_name = "metahand";
	if ( !n.getParam(addNodeName("hand"), hand_name) )
		ROS_WARN ( "[SquirrelObjectManipulationServer] No input hand name, using default %s", hand_name.c_str() );

	ROS_INFO ( "Starting squirrel object manipulation server with parameters: action_name = %s, hand_name = %s", action_name.c_str(), hand_name.c_str() );

	// Create the server
	SquirrelObjectManipulationServer object_manipulation_server ( n, action_name );
	if ( !object_manipulation_server.initialize (hand_name) )
	{
		ROS_WARN ( "[SquirrelObjectManipulationServer] Could not initialize" );
		ros::shutdown ;
		return EXIT_FAILURE;
	}
	// Lsten to action calls
	ros::spin();

	// Shutdown and exit
	ros::shutdown();
	return EXIT_SUCCESS;
}
