#ifndef SQUIRREL_GRASP_SERVER_H_
#define SQUIRREL_GRASP_SERVER_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

/**
 * \brief Grasp server
 * \author Timothy Patten (patten@acin.tuwien.ac.at)
 */
class SquirrelGraspServer
{
public:

  /**
   * \brief Constructor with ros node handle
   */
  SquirrelGraspServer ( ros::NodeHandle &n );

  /**
   * \brief Destructor
   */
  virtual
  ~SquirrelGraspServer ();

private:

  // ROS
  ros::NodeHandle *n_;

};

#endif

