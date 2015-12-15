squirrel_manipulation
=====================

Technical Maintainer: lokalmatador (Philipp Zech, University of Innsbruck)

Repository for kinesthetic teaching.

## Starting the node in simulation

```bash 
$ roslaunch squirrel_kinesthetic_teaching squirrel_kinesthetic_teaching_node_sim.launch
``` 

## Starting the node on the robot

```bash 
$ roslaunch squirrel_kinesthetic_teaching squirrel_kinesthetic_teaching_node_real.launch
``` 

## Start teaching
```bash 
$ rostopic pub /squirrel_manipulation/start_teaching std_msgs/String "data: 'start'" 
``` 

## Stop teaching
```bash 
$ rostopic pub /squirrel_manipulation/stop_teaching std_msgs/String "data: 'stop'" 
``` 


