cmake_minimum_required(VERSION 2.8.3)
project(xview_slam)

#<<<<<<< HEAD
#include_directories(
# include
#  ${catkin_INCLUDE_DIRS}
#  ${GTSAM_INCLUDE_DIRS}
#  ${EIGEN_INCLUDE_DIRS}
#  /home/emilio/catkin_ws/devel/include/eigen3
#)
#||||||| merged common ancestors
#include_directories(
# include
#  ${catkin_INCLUDE_DIRS}
#   ${GTSAM_INCLUDE_DIRS}
#)
#=======
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native")
#>>>>>>> origin/feature/review_1

find_package(catkin_simple 0.1.0 REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_executable(odometry_factors_handler src/odometry_factors_handler.cc)
target_link_libraries(odometry_factors_handler ${catkin_LIBRARIES} ${PROJECT_NAME})

cs_install()
cs_export()
