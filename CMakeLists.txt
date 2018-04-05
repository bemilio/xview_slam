cmake_minimum_required(VERSION 2.8.3)
project(xview_slam)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native")

find_package(catkin_simple 0.1.0 REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_executable(odometry_factors_handler src/odometry_factors_handler.cc)
target_link_libraries(odometry_factors_handler ${catkin_LIBRARIES} ${PROJECT_NAME})

cs_install()
cs_export()