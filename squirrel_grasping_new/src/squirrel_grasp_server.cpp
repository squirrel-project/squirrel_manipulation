#include "squirrel_grasping_new/squirrel_grasp_server.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
SquirrelGraspServer::SquirrelGraspServer ( ros::NodeHandle &n ) :
  n_ ( new ros::NodeHandle(n) )
{}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
SquirrelGraspServer::~SquirrelGraspServer ()
{}


int
main ( int argc, char **argv )
{
  ros::init(argc, argv, "squirrel_grasp_server");
  ros::NodeHandle n ( "~" );

  ROS_INFO ( "Squirrel grasp server started" );

  // Create segmenter
  SquirrelGraspServer *grasp_server = new SquirrelGraspServer ( n );


  ros::shutdown ();
  return EXIT_SUCCESS;
}
