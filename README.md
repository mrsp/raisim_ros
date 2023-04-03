# README
 This repository is a ROS bridge with [Raisim](www.raisim.com) for real-time simulation

# Install
* git clone https://github.com/mrsp/raisim_ros.git into a ROS workspace
* catkin_make -DCMAKE_PREFIX_PATH=${LOCAL_INSTALL}
* If raisimConfig.cmake is not Found configure list(APPEND CMAKE_PREFIX_PATH ${ENV_VAR}/raisim) in CMakeLists.txt by changing raisim to the name of the folder where raisim is built.

# NAO robot simulation
* roslaunch raisim_ros nao.launch
# ATLAS humanoid simulation
* roslaunch raisim_ros atlas.launch 
