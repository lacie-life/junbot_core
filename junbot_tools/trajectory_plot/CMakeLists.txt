cmake_minimum_required(VERSION 2.8.3)

project(junbot_plotter)

find_package(catkin REQUIRED
    COMPONENTS message_generation rospy tf std_msgs geometry_msgs nav_msgs)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#   DEPENDENCIES std_msgs
#)


# Declare that this catkin package's runtime dependencies
catkin_package(
    INCLUDE_DIRS
    LIBRARIES ${PROJECT_NAME} 
    CATKIN_DEPENDS message_runtime rospy
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
   src/generate_plots.py
   src/data_odometry.py
   src/path.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE) 
