squirrel_manipulation
=====================

Technical Maintainer: shangl (Simon Hangl, University of Innsbruck)
[never commit to the main repository. Instead send pull request to the technical maintainer --> shangl/manipulation]

Repository for manipulation related SQUIRREL packages.

## Starting object manipulation actions

```bash 
$ roslaunch squirrel_object_manipulation  squirrel_object_manipulation.launch
``` 

## Testing push action with simulator

Start neccessary packages with: 

```bash 
$ roslaunch squirrel_object_manipulation  test_push_sim.launch 
``` 

This launch file starts: 
- simualator
- navigation package + rviz
- fake perception
- keyboard controller

Localize robot by moving with keyboard controller (WASD for translation, QE for rotation)

To set goal for /push action use use actionlib

```bash 
$ rosrun actionlib axclient.py /push
``` 
