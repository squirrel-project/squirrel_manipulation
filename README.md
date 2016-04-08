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


set up environment variable (add to .bashrc):
export KUKADU_HOME=PATH_TO_SQUIRREL_CATKIN/src/kukadu
