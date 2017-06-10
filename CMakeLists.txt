cmake_minimum_required(VERSION 2.8.3)
project(tue_hsr)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tue_hsr
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

