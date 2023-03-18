
cmake_minimum_required(VERSION 3.0.2)
project(junbot_adaptive)

## Find catkin and any catkin packages
<<<<<<< HEAD
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg move_base_msgs  custom_msgs geometry_msgs vision_msgs)
=======
find_package(catkin REQUIRED COMPONENTS tf roscpp rospy std_msgs genmsg move_base_msgs  custom_msgs geometry_msgs)
>>>>>>> 4788f1ac00b49a177e13d0065c95c74de72c0a28

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

add_executable(subGridMap src/subGridMap.cpp)
target_link_libraries(subGridMap ${catkin_LIBRARIES})
add_dependencies(subGridMap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(adaptiveControl src/adaptiveControl.cpp)
target_link_libraries(adaptiveControl ${catkin_LIBRARIES})
add_dependencies(adaptiveControl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

<<<<<<< HEAD
=======
add_executable(objectLayer src/ObjectLayer.cpp)
target_link_libraries(objectLayer ${catkin_LIBRARIES})
add_dependencies(objectLayer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(subscribeTraj src/subscribeTraj.cpp)
target_link_libraries(subscribeTraj ${catkin_LIBRARIES})
add_dependencies(subscribeTraj ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
>>>>>>> 4788f1ac00b49a177e13d0065c95c74de72c0a28

