cmake_minimum_required(VERSION 3.0.2)
project(franka_safety)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    roscpp
    rospy
    std_msgs
    geometry_msgs
    visualization_msgs
    franka_interface
    franka_core_msgs
)

catkin_package()


catkin_python_setup()
catkin_install_python(PROGRAMS scripts/virtual_wall.py 
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


include_directories(
# include
    ${catkin_INCLUDE_DIRS}
)

