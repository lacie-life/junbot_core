cmake_minimum_required(VERSION 3.0.2)
project(junbot_adaptive)

option(TEST_BUILD "Build test" OFF)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
        tf
        roscpp
        rospy
        std_msgs
        genmsg
        move_base_msgs
        custom_msgs
        geometry_msgs
        vision_msgs
        )

## Declare a catkin package
catkin_package(
        CATKIN_DEPENDS custom_msgs geometry_msgs)

## Build talker and listener
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(junbot_planner
        src/junbot_planner.cpp
        src/SemanticPlanner.cpp
        )
target_link_libraries(junbot_planner ${catkin_LIBRARIES})
add_dependencies(junbot_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

################################################################################
# Test
################################################################################
if (TEST_BUILD)
    add_executable(cancel_mission test/cancel_mission.cpp)
    target_link_libraries(cancel_mission ${catkin_LIBRARIES})
    add_dependencies(cancel_mission ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

    add_executable(add_layer test/add_layer.cpp)
    target_link_libraries(add_layer ${catkin_LIBRARIES})
    add_dependencies(add_layer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

    add_executable(subGridMap test/subGridMap.cpp)
    target_link_libraries(subGridMap ${catkin_LIBRARIES})
    add_dependencies(subGridMap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

    add_executable(adaptiveControl test/adaptiveControl.cpp)
    target_link_libraries(adaptiveControl ${catkin_LIBRARIES})
    add_dependencies(adaptiveControl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

    add_executable(objectLayer test/ObjectLayer.cpp)
    target_link_libraries(objectLayer ${catkin_LIBRARIES})
    add_dependencies(objectLayer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

    add_executable(subscribeTraj test/subscribeTraj.cpp)
    target_link_libraries(subscribeTraj ${catkin_LIBRARIES})
    add_dependencies(subscribeTraj ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endif (TEST_BUILD)

################################################################################
# Install
################################################################################
install(TARGETS junbot_planner
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

if (TEST_BUILD)
    install(TARGETS add_layer
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            )
    install(TARGETS subGridMap
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            )
    install(TARGETS adaptiveControl
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            )
    install(TARGETS objectLayer
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            )
    install(TARGETS subscribeTraj
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            )
endif (TEST_BUILD)

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

# install(DIRECTORY bag config launch rviz
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
