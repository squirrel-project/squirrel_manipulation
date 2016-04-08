squirrel_manipulation_waypoint
==============================

Technical Maintainer: lokalmatador (Philipp Zech, University of Innsbruck)

Package for waypoint generation.

## Starting the node

```bash 
$ roslaunch squirrel_manipulation_waypoint squirrel_manipulation_waypoint_node.launch
``` 

## Request waypoints
```bash 
$ $ rosservice call /squirrel_manipulation_waypoints/get_waypoints squirrel_waypoints_msgs/ManipulationWaypointServiceRequest
``` 

## Requirements
* numpy


