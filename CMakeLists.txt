cmake_minimum_required(VERSION 2.8.3)
project(xview_slam)
add_definitions(-std=c++11)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
)
find_package(GTSAMCMakeTools)
find_package(GTSAM REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imu_listener
#  CATKIN_DEPENDS ros std_msgs
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  /home/emilio/catkin_ws/devel/include/eigen3
)

## Declare a C++ library
#add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/src/imu_listener.cpp
#)


add_executable(imu_listener src/imu_listener.cpp)
target_link_libraries(imu_listener gtsam ${catkin_LIBRARIES})

