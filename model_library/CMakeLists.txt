###########
## Build ##
###########
cmake_minimum_required(VERSION 3.0.2)
project(model_library)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
  # DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/model.cpp
)

##确保可以链接到catkin库
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

# ## Mark libraries for installation
# ## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# ## Mark other files for installation (e.g. launch and bag files, etc.)
# ##在安装你的 ROS 包时，将 include/ 目录中的所有文件复制到 ROS 安装的标准头文件目录中。
# ##这样做的目的是让其他包能够在它们自己的构建过程中找到并使用这些头文件。
# install(
#   DIRECTORY include/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )