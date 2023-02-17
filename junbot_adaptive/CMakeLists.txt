# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 3.0.2)
project(junbot_adaptive)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg move_base_msgs  custom_msgs geometry_msgs)

## Declare a catkin package
catkin_package(
    CATKIN_DEPENDS custom_msgs geometry_msgs)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(cancel_mission src/cancel_mission.cpp)
target_link_libraries(cancel_mission ${catkin_LIBRARIES})
add_dependencies(cancel_mission ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(add_layer src/add_layer.cpp)
target_link_libraries(add_layer ${catkin_LIBRARIES})
add_dependencies(add_layer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# %EndTag(FULLTEXT)%
