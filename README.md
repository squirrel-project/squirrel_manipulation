squirrel_manipulation
=====================

Technical Maintainer: shangl (Simon Hangl, University of Innsbruck)
[never commit to the main repository. Instead send pull request to the technical maintainer --> shangl/manipulation]

Repository for manipulation related SQUIRREL packages.

## Starting object manipulation actions

```bash 
$ roslaunch squirrel_object_manipulation  squirrel_object_manipulation.launch
``` 

============installing kukadu=========================

clone this package with

git clone --recursive $URL_TO_GIT_REPO$

install dependencies for kukadu:
sudo apt-get install ros-indigo-desktop-full libgsl0-dev gnuplot gnuplot-x11 libarmadillo-dev libboost-all-dev libncurses5-dev libarmadillo-dev libgsl0-dev gnuplot gnuplot-x11 libarmadillo-dev libboost-all-dev libncurses5-dev libarmadillo-dev liballegro5-dev ros-indigo-pcl-ros ros-indigo-moveit-ros-planning-interface python3.4-dev

set up environment variable (add to .bashrc):
export KUKADU_HOME=PATH_TO_SQUIRREL_CATKIN/src/kukadu

