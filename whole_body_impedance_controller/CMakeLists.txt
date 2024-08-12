cmake_minimum_required(VERSION 3.0.2)
project(whole_body_impedance_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  trajectory_msgs
  message_generation
  eigen_conversions
  model_library
)

find_package(Eigen3 REQUIRED)


###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
 CATKIN_DEPENDS geometry_msgs roscpp std_msgs trajectory_msgs
 DEPENDS
)

###########
## Build ##
###########

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node 
              src/whole_body_impedance_controller.cpp
              src/whole_body_impedance_controller_node.cpp)
target_link_libraries(${PROJECT_NAME}_node  ${catkin_LIBRARIES})

