Squirrel_manipulation
=====================

Technical Maintainer: shangl (Simon Hangl, University of Innsbruck)
[never commit to the main repository. Instead send pull request to the technical maintainer --> shangl/manipulation]

Repository for manipulation related SQUIRREL packages.

## Starting object manipulation actions

```bash 
$ roslaunch squirrel_object_manipulation  squirrel_object_manipulation.launch
``` 

Planning over the ros interface
====================================
- start up the robotino control stack + moveit
	- roslaunch robotino_bringup robot.launch
	- roslaunch uibk_moveit_config moveit_planning.launch
	- rosrun squirrel_ptp_server squirrel_ptp
- there will be now 2 action servers
	- cartesian_ptp: planning in Cartesian space (currently not working because of an overloaded urdf model)
	- joint_ptp: planning in 8-dof joint space (x y theta joint1 joint2 joint3 joint4 joint5)

Installation prerequisites for kukadu
----------------------------------------
There is dependency for kukadu package. 
To install kukadu package:
- install dependencies listed below are needed. 
 -Clone kukadu package in your catkin_ws/src  with
```bash 
git clone --recursive https://github.com/squirrel-project/kukadu
``` 
- set up environment variable (add to .bashrc):
export KUKADU_HOME=PATH_TO_SQUIRREL_CATKIN/src/kukadu

Installation dependencies for kukadu:
```bash 
sudo apt-get install libgsl0-dev gnuplot gnuplot-x11 libarmadillo-dev \
libboost-all-dev libncurses5-dev libarmadillo-dev liballegro5-dev \
ros-indigo-pcl-ros ros-indigo-moveit-ros-planning-interface \
python3.4-dev liblapacke-dev gtk+2.0 \
bison \
build-essential \
cmake \
doxygen \
fabric \
flex \
freeglut3-dev \
g++ \
gcc \
gfortran \
git-core \
gnuplot \
graphviz-dev \
libann-dev \
libcv-dev \
libcvaux-dev \
libdc1394-22-dev \
libf2c2-dev \
libgtest-dev \
libgtkglext1-dev \
libhighgui-dev \
liblapack-dev \
libplib-dev \
libqhull-dev \
libsdl1.2-dev \
libx11-dev \
libx11-dev \
libxi-dev \
libxmu-dev \
make \
meld \
python-nose \
python-unittest2 \
realpath \
regexxer \
swig2.0 \
tcl8.5-dev \
tk-dev \
tk8.5-dev \
libfreenect-dev \
``` 


