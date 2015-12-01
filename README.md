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

download kukadu to src/
git@github.com:shangl/kukadu.git

install dependencies for kukadu:
sudo apt-get install ros-indigo-desktop-full libgsl0-dev gnuplot gnuplot-x11 libarmadillo-dev libboost-all-dev libncurses5-dev libarmadillo-dev

set up environment variable (add to .bashrc):
export KUKADU_HOME=PATH_TO_SQUIRREL_CATKIN/src/kukadu
