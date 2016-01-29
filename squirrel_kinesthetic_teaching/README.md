squirrel_kinesthetic_teaching
=============================

Technical Maintainer: lokalmatador (Philipp Zech, University of Innsbruck)

Package for kinesthetic teaching.

## Starting the node in simulation

```bash 
$ roslaunch squirrel_kinesthetic_teaching squirrel_kinesthetic_teaching_node_simulator.launch
``` 

## Starting the node on the robot

```bash 
$ roslaunch squirrel_kinesthetic_teaching squirrel_kinesthetic_teaching_node_robot.launch
``` 

## Start teaching
```bash 
$ rosservice call /squirrel_manipulation/start_teaching "{}"
``` 

## Stop teaching
```bash 
$ $ rosservice call /squirrel_manipulation/stop_teaching "{}"
``` 


