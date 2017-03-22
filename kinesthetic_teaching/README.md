<a id="top"/> 
#kinesthetic_teaching

**This package is a demo for using kinesthetic teaching in KUKADU.

Technical Maintainer: -

##Contents

1. <a href="#1--installation-requirements">Installation Requirements</a>
2. <a href="#2--execution">Execution</a>
3. <a href="#3--software-architecture">Software architecture</a>


## 1. Installation Requirements: <a id="1--installation-requirements"/> 

Dependencies: geometry_msgs roscpp std_msgs squirrel_sensing_node kukadu 


## 2. Execution: <a id="2--execution"/> 

Execution:

To run the kinesthetic teaching demo, you have to start the robotino 8-dof driver:

```
roslaunch robotino_bringup robot.launch
```

Then, you have to launch the MoveIt model in the background:

```
roslaunch uibk_moveit_config moveit_planning.launch

```
Wait until the model is loaded..

Afterwards, run the sensing node to get readings from the F/T sensor.

```
rosrun squirrel_sensing_node sensing 

```

Also, to map the sensor output from type std_msgs/Float64MultiArray at topic /wrist to type geometry_msgs/Wrench at topic real/robotino/sensoring/cartesian_wrench, run the conversion node:

```
rosrun kinesthetic_teaching sensor_node
```

Finally, to run the kinesthetic teaching demo, execute:

```
rosrun kinesthetic_teaching kinesthetic_teaching_DEMO 

```

The demo includes teaching, recording and playing functionalities.



