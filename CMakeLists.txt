cmake_minimum_required(VERSION 3.0.2)
project(raisim_ros)


list(APPEND CMAKE_PREFIX_PATH /home/michael/raisim_build)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
find_package(Boost REQUIRED COMPONENTS system)



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
)

find_package(raisim CONFIG REQUIRED)
find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES raisim_ros
  CATKIN_DEPENDS std_msgs roscpp rospy sensor_msgs nav_msgs geometry_msgs
  DEPENDS EIGEN3
)

include_directories (include ${EIGEN3_INCLUDE_DIRS}  ${catkin_INCLUDE_DIRS} ${LOCAL_INSTALL})


add_executable(nao_ros src/nao_ros.cpp )
target_link_libraries(nao_ros PUBLIC raisim::raisim pthread ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${Boost_LIBRARIES})
add_executable(atlas_ros src/atlas_ros.cpp )
target_link_libraries(atlas_ros PUBLIC raisim::raisim pthread ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${Boost_LIBRARIES})
add_executable(talos_ros src/talos_ros.cpp )
target_link_libraries(talos_ros PUBLIC raisim::raisim pthread ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${Boost_LIBRARIES})
add_executable(morphLynX_ros src/morphLynX_ros.cpp )
target_link_libraries(morphLynX_ros PUBLIC raisim::raisim pthread ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${Boost_LIBRARIES})
add_executable(digit_ros src/digit_ros.cpp )
target_link_libraries(digit_ros PUBLIC raisim::raisim pthread ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${Boost_LIBRARIES})

