#include "squirrel_grasping_new/squirrel_grasp_server.h"

using namespace std;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
SquirrelGraspServer::SquirrelGraspServer ( ros::NodeHandle &n, const std::string &action_name ) :
  n_ ( new ros::NodeHandle(n) ),
  action_name_ ( action_name ),
  as_ ( *n_, action_name, boost::bind(&SquirrelGraspServer::graspCallBack, this, _1), false)
{
  as_.start();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
SquirrelGraspServer::~SquirrelGraspServer ()
{}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelGraspServer::initialize ( const std::string &hand_name )
{
  if ( hand_name.compare(METAHAND_STRING_) == 0 )
  {
    hand_type_ = SquirrelGraspServer::METAHAND;
    ROS_INFO ( "[SquirrelGraspServer::initialize] Metahand selected" );
  }
  else if ( hand_name.compare(SOFTHAND_STRING_) == 0 )
  {
    hand_type_ = SquirrelGraspServer::SOFTHAND;
    ROS_INFO ( "[SquirrelGraspServer::initialize] Softhand selected" );
  }
  else
  {
    hand_type_ = SquirrelGraspServer::UNKNOWN;
    ROS_ERROR ( "[SquirrelGraspServer::initialize] Unknown hand name %s ", hand_name.c_str() );
    return false;
  }

  arm_unfold_client_ = new ros::ServiceClient ( n_->serviceClient<squirrel_motion_planner_msgs::UnfoldArm>("/squirrel_8dof_planning/unfold_arm") );
  arm_end_eff_planner_client_ = new ros::ServiceClient ( n_->serviceClient<squirrel_motion_planner_msgs::PlanEndEffector>("/squirrel_8dof_planning/find_plan_end_effector") );
  arm_pose_planner_client_ = new ros::ServiceClient ( n_->serviceClient<squirrel_motion_planner_msgs::PlanPose>("/squirrel_8dof_planning/find_plan_pose") );
  arm_send_trajectory_client_ = new ros::ServiceClient ( n_->serviceClient<squirrel_motion_planner_msgs::SendControlCommand>("/squirrel_8dof_planning/send_trajectory_controller") );
  //hand_client_ = new ros::ServiceClient ( n_->serviceClient<squirrel_motion_planner_msgs::PlanEndEffector>("/squirrel_8dof_planning/find_plan_end_effector") );

  // Get the folded and unfolded poses from the parameter server
  vector<double> loaded_poses;
  if ( !n_->getParam("/squirrel_grasp_server/trajectory_folding_arm", loaded_poses) )
  {
    ROS_ERROR ( "[SquirrelGraspServer::initialize] Could not retrieve unfolded poses " );
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
    ROS_ERROR ( "[SquirrelGraspServer::initialize] Parameter list 'trajectory_folding_arm' is not divisible by 5, folding arm trajectory has not been loaded" );
    return false;
  }

  current_joints_.resize ( 8 );  // 5 for arm and 3 for base
  joints_sub_ = n_->subscribe ( "/arm_controller/joint_trajectory_controller/state", 1, &SquirrelGraspServer::jointsCallBack, this );

  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SquirrelGraspServer::graspCallBack ( const squirrel_manipulation_msgs::BlindGraspGoalConstPtr &goal )
{
  ROS_INFO ( "[SquirrelGraspServer::graspCallBack] Called with parameters" );

  bool success = false;

  // Transform to hand_wrist_link frame
  geometry_msgs::PoseStamped transformed_goal;
  string goal_frame = goal->heap_center_pose.header.frame_id;
  geometry_msgs::PoseStamped goal_pose = goal->heap_center_pose;
  transformPose ( goal_frame, PLANNING_FRAME_, goal_pose, transformed_goal );

  if ( hand_type_ == SquirrelGraspServer::METAHAND )
    success = metahandCallBack ( transformed_goal );
  else if ( hand_type_ == SquirrelGraspServer::SOFTHAND )
    success = softhandCallBack ( transformed_goal );

  if ( success )
  {
    ROS_INFO ( "[SquirrelGraspServer::graspCallBack] Succeeded" );
    // set the action state to succeeded
    as_.setSucceeded ( result_ );
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelGraspServer::metahandCallBack ( const geometry_msgs::PoseStamped &goal )
{
  // Open hand
  feedback_.current_phase = "opening hand";
  feedback_.current_status = "starting";
  as_.publishFeedback ( feedback_ );
  // Call service ...
  feedback_.current_status = "success";
  as_.publishFeedback ( feedback_ );

  // Plan arm to unfolded position
  feedback_.current_phase = "unfolding";
  feedback_.current_status = "starting";
  as_.publishFeedback ( feedback_ );
  // If the arm is folded then use the unfolding service
  if ( armIsFolded() )
  {
    unfold_goal_.request.check_octomap_collision = true;
    unfold_goal_.request.check_self_collision = true;
    ROS_INFO ( "[SquirrelGraspServer::metahandCallBack] Unfolding arm" );
    if ( !arm_unfold_client_->call(unfold_goal_) )
    {
      ROS_WARN ( "[SquirrelGraspServer::metahandCallBack] Failed to unfold the arm using unfolding service" );
      // Set the result to something here
      feedback_.current_status = "failed";
      return false;
    }
    if ( unfold_goal_.response.result != 0 )
    {
      ROS_WARN ( "[SquirrelGraspServer::metahandCallBack] Unfold service returned with result %i", unfold_goal_.response.result  );
      // Set the result to something here
      feedback_.current_status = "failed";
      return false;
    }
    else
    {
      ROS_INFO ( "[SquirrelGraspServer::metahandCallBack] Unfolding service returned with 0" );
    }
    ros::Duration(2.0).sleep();
  }
  // Otherwise use the joint planner
  else
  {
    vector<double> goal_joints ( 8 );  // [basex basey basez arm_joint1 arm_joint2 arm_joint3 arm_joint4 arm_joint5]
    goal_joints[0] = current_joints_[0];
    goal_joints[1] = current_joints_[1];
    goal_joints[2] = current_joints_[2];
    goal_joints[3] = unfolded_pose[0];
    goal_joints[4] = unfolded_pose[1];
    goal_joints[5] = unfolded_pose[2];
    goal_joints[6] = unfolded_pose[3];
    goal_joints[7] = unfolded_pose[4];
    pose_goal_.request.joints = goal_joints;
    pose_goal_.request.max_planning_time = 3.0;
    pose_goal_.request.check_octomap_collision = true;
    pose_goal_.request.check_self_collision = true;
    pose_goal_.request.fold_arm = false;
    pose_goal_.request.min_distance_before_folding = 0.0;
    if ( !arm_pose_planner_client_->call(pose_goal_) )
    {
      ROS_WARN ( "[SquirrelGraspServer::metahandCallBack] Failed to plan to unfolded position [%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f]",
                 goal_joints[0], goal_joints[1], goal_joints[2], goal_joints[3],
          goal_joints[4], goal_joints[5], goal_joints[6], goal_joints[7] );
      // Set the result to something here
      feedback_.current_status = "failed";
      return false;
    }
    if ( pose_goal_.response.result != 0 )
    {
      ROS_WARN ( "[SquirrelGraspServer::metahandCallBack] Planning to unfolded position returned with result %i", pose_goal_.response.result  );
      // Set the result to something here
      feedback_.current_status = "failed";
      return false;
    }
    else
    {
      ROS_INFO ( "[SquirrelGraspServer::metahandCallBack] Planning to unfolded position returned with 0" );
    }
    ROS_INFO ( "[SquirrelGraspServer::metahandCallBack] Moving to unfolded position" );
    if ( !arm_send_trajectory_client_->call(cmd_goal_) )
    {
      ROS_WARN ( "[SquirrelGraspServer::metahandCallBack] Failed to send command to unfold" );
      // Set the result to something here
      feedback_.current_status = "failed";
      return false;
    }
    // Sleep
    ros::Duration(2.0).sleep();
    if ( cmd_goal_.response.result != 0 )
    {
      ROS_WARN ( "[SquirrelGraspServer::metahandCallBack] Sending command to move to unfolded position returned with result %i", cmd_goal_.response.result  );
      // Set the result to something here
      feedback_.current_status = "failed";
      return false;
    }
    else
    {
      ROS_INFO ( "[SquirrelGraspServer::metahandCallBack] Sending command to move to unfolded position returned with 0" );
    }
  }
  // Publish feedback
  feedback_.current_status = "success";
  as_.publishFeedback ( feedback_ );

  // Plan end effector to goal position
  //rosservice call /squirrel_8dof_planning/find_plan_end_effector "check_octomap_collision: true
  feedback_.current_phase = "grasping";
  feedback_.current_status = "starting";
  as_.publishFeedback ( feedback_ );
  vector<double> goal_pose ( 6 );  // [x y z roll pitch yaw]
  goal_pose[0] = goal.pose.position.x;
  goal_pose[1] = goal.pose.position.y;
  goal_pose[2] = goal.pose.position.z;
  goal_pose[3] = 0;
  goal_pose[4] = 0;
  goal_pose[5] = 0;
  end_eff_goal_.request.positions = goal_pose;
  end_eff_goal_.request.max_planning_time = 3.0;
  end_eff_goal_.request.check_octomap_collision = true;
  end_eff_goal_.request.check_self_collision = true;
  end_eff_goal_.request.fold_arm = false;
  end_eff_goal_.request.min_distance_before_folding = 0.0;
  if ( !arm_end_eff_planner_client_->call(end_eff_goal_) )
  {
    ROS_WARN ( "[SquirrelGraspServer::metahandCallBack] Failed to find a plan to pose [%.2f %.2f %.2f %.2f %.2f %.2f]",
               goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, 0.0, 0.0, 0.0 );
    // Set the result to something here
    feedback_.current_status = "failed";
    return false;
  }
  if ( end_eff_goal_.response.result != 0 )
  {
    ROS_WARN ( "[SquirrelGraspServer::metahandCallBack] Planning to pose returned with result %i", end_eff_goal_.response.result  );
    // Set the result to something here
    feedback_.current_status = "failed";
    return false;
  }
  else
  {
    ROS_INFO ( "[SquirrelGraspServer::metahandCallBack] Planning to pose returned with 0" );
  }
  ROS_INFO ( "[SquirrelGraspServer::metahandCallBack] Moving to pose" );
  if ( !arm_send_trajectory_client_->call(cmd_goal_) )
  {
    ROS_WARN ( "[SquirrelGraspServer::metahandCallBack] Failed to send command to pose" );
    // Set the result to something here
    feedback_.current_status = "failed";
    return false;
  }
  // Sleep
  ros::Duration(2.0).sleep();
  if ( cmd_goal_.response.result != 0 )
  {
    ROS_WARN ( "[SquirrelGraspServer::metahandCallBack] Sending command to move to pose returned with result %i", cmd_goal_.response.result  );
    // Set the result to something here
    feedback_.current_status = "failed";
    return false;
  }
  else
  {
    ROS_INFO ( "[SquirrelGraspServer::metahandCallBack] Sending command to move to pose returned with 0" );
  }
  feedback_.current_status = "success";
  as_.publishFeedback ( feedback_ );

  // Close hand
  feedback_.current_phase = "closing hand";
  feedback_.current_status = "starting";
  as_.publishFeedback ( feedback_ );
  // Call service ...
  feedback_.current_status = "success";
  as_.publishFeedback ( feedback_ );

  // Plan arm to unfolded position
  vector<double> goal_joints ( 8 );  // [basex basey basez arm_joint1 arm_joint2 arm_joint3 arm_joint4 arm_joint5]
  goal_joints[0] = current_joints_[0];
  goal_joints[1] = current_joints_[1];
  goal_joints[2] = current_joints_[2];
  goal_joints[3] = unfolded_pose[0];
  goal_joints[4] = unfolded_pose[1];
  goal_joints[5] = unfolded_pose[2];
  goal_joints[6] = unfolded_pose[3];
  goal_joints[7] = unfolded_pose[4];
  pose_goal_.request.joints = goal_joints;
  pose_goal_.request.max_planning_time = 3.0;
  pose_goal_.request.check_octomap_collision = true;
  pose_goal_.request.check_self_collision = true;
  pose_goal_.request.fold_arm = false;
  pose_goal_.request.min_distance_before_folding = 0.0;
  if ( !arm_pose_planner_client_->call(pose_goal_) )
  {
    ROS_WARN ( "[SquirrelGraspServer::metahandCallBack] Failed to return arm to unfolded position [%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f]",
               goal_joints[0], goal_joints[1], goal_joints[2], goal_joints[3],
        goal_joints[4], goal_joints[5], goal_joints[6], goal_joints[7] );
    // Set the result to something here
    feedback_.current_status = "failed";
    return false;
  }
  if ( pose_goal_.response.result != 0 )
  {
    ROS_WARN ( "[SquirrelGraspServer::metahandCallBack] Planning to return to arm to unfolded position returned with result %i", pose_goal_.response.result  );
    // Set the result to something here
    feedback_.current_status = "failed";
    return false;
  }
  else
  {
    ROS_INFO ( "[SquirrelGraspServer::metahandCallBack] Planning to return arm to unfolded position returned with 0" );
  }
  ROS_INFO ( "[SquirrelGraspServer::metahandCallBack] Returning to unfolded position" );
  if ( !arm_send_trajectory_client_->call(cmd_goal_) )
  {
    ROS_WARN ( "[SquirrelGraspServer::metahandCallBack] Failed to send command to return arm" );
    // Set the result to something here
    feedback_.current_status = "failed";
    return false;
  }
  // Sleep
  ros::Duration(2.0).sleep();
  if ( cmd_goal_.response.result != 0 )
  {
    ROS_WARN ( "[SquirrelGraspServer::metahandCallBack] Sending command to return arm to unfolded position returned with result %i", cmd_goal_.response.result  );
    // Set the result to something here
    feedback_.current_status = "failed";
    return false;
  }
  else
  {
    ROS_INFO ( "[SquirrelGraspServer::metahandCallBack] Sending command to return arm to unfolded position returned with 0" );
  }

  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelGraspServer::softhandCallBack ( const geometry_msgs::PoseStamped &goal )
{
  feedback_.current_phase = "opening hand";
  feedback_.current_status = "success";
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SquirrelGraspServer::jointsCallBack ( const control_msgs::JointTrajectoryControllerStateConstPtr &joints )
{
  // ROS_INFO ( "[SquirrelGraspServer::jointsCallBack] Message received" );
  std::vector<std::string>::const_iterator found;
  int idx;
  for ( size_t i = 0; i < joints->joint_names.size(); ++i )
  {
    if ( joints->joint_names[i].compare("base_x") == 0 )
      current_joints_[0] = joints->actual.positions[i];
    else if ( joints->joint_names[i].compare("base_y") == 0 )
      current_joints_[1] = joints->actual.positions[i];
    else if ( joints->joint_names[i].compare("base_z") == 0 )
      current_joints_[2] = joints->actual.positions[i];
    else if ( joints->joint_names[i].compare("arm_joint1") == 0 )
      current_joints_[3] = joints->actual.positions[i];
    else if ( joints->joint_names[i].compare("arm_joint2") == 0 )
      current_joints_[4] = joints->actual.positions[i];
    else if ( joints->joint_names[i].compare("arm_joint3") == 0 )
      current_joints_[5] = joints->actual.positions[i];
    else if ( joints->joint_names[i].compare("arm_joint4") == 0 )
      current_joints_[6] = joints->actual.positions[i];
    else if ( joints->joint_names[i].compare("arm_joint5") == 0 )
      current_joints_[7] = joints->actual.positions[i];
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelGraspServer::transformPose ( const string &origin_frame, const string &target_frame,
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
    ROS_ERROR ( "[SquirrelGraspServer::transformPose] Tf listener exception thrown with message '%s'",ex.what() );
    ros::Duration(1.0).sleep();
    return false;
  }
  // Print out successful transformation
  ROS_INFO ( "[SquirrelGraspServer::transformPose] Transformed from: \n[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
             in.pose.position.x, in.pose.position.y, in.pose.position.z, in.pose.orientation.x, in.pose.orientation.y, in.pose.orientation.z, in.pose.orientation.w,
             out.pose.position.x, out.pose.position.y, out.pose.position.z, out.pose.orientation.x, out.pose.orientation.y, out.pose.orientation.z, out.pose.orientation.w );
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
vector<double> SquirrelGraspServer::poseDiff ( const vector<double> &pose1, const vector<double> &pose2 ) const
{
  vector<double> diff;
  // Cannot compare if pose vectors are not the same size
  if ( pose1.size() != pose2.size() )
  {
    ROS_ERROR ( "[SquirrelGraspServer::poseDiff] Cannot compare inputs with different sizes, %lu %lu", pose1.size(), pose2.size() );
    return diff;
  }

  // Take the absolute difference between the elements
  diff.resize ( pose1.size() );
  for ( size_t i = 0; i < diff.size(); ++i )
    diff[i] = fabs ( pose1[i] - pose2[i] );

  return diff;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelGraspServer::armIsFolded () const
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
  double threshold = 0.139626; // 8 degrees
  for ( size_t i = 0; i < diff.size(); ++i )
  {
    // If one joint is above the threshold then arm is not in the pose
    if ( diff[i] > threshold )
      return false;
  }
  // All joints are near
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool SquirrelGraspServer::armIsUnfolded () const
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
  double threshold = 0.139626; // 8 degrees
  for ( size_t i = 0; i < diff.size(); ++i )
  {
    // If one joint is above the threshold then arm is not in the pose
    if ( diff[i] > threshold )
      return false;
  }
  // All joints are near
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main ( int argc, char **argv )
{
  ros::init ( argc, argv, "squirrel_grasp_server" );
  ros::NodeHandle n ( "" );

  string action_name = "squirrel_grasp_server";
  if ( !n.getParam("action_name", action_name) )
    ROS_WARN ( "[SquirrelGraspServer] No input action name, using default %s", action_name.c_str() );
  string hand_name = "metahand";
  if ( !n.getParam("hand", hand_name) )
    ROS_WARN ( "[SquirrelGraspServer] No input hand name, using default %s", hand_name.c_str() );

  ROS_INFO ( "Starting squirrel grasp server with parameters: action_name = %s, hand_name = %s", action_name.c_str(), hand_name.c_str() );

  // Create segmenter
  SquirrelGraspServer grasp_server ( n, action_name );
  if ( !grasp_server.initialize (hand_name) )
  {
    ROS_WARN ( "[SquirrelGraspServer] Could not initialize" );
    ros::shutdown ;
    return EXIT_FAILURE;
  }
  // Otherwise listen to action calls
  ros::spin();

  // Shutdown and exit
  ros::shutdown();
  return EXIT_SUCCESS;
}
