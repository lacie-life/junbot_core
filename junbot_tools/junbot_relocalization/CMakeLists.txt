cmake_minimum_required(VERSION 3.0.2)
project(junbot_relocalization)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        )

#list(APPEND CMAKE_PREFIX_PATH "~/Github/libtorch/")

#find_package(Torch REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED NO_MODULE)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES junbot_relocalization
        CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
)

include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        #${TORCH_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS})

# file(GLOB HEADERS
#         ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
#         ${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp
#         )
# file(GLOB SOURCES
#         ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
#         )

# source_group("Header Files" FILES ${HEADERS})
# source_group("Source Files" FILES ${SOURCES})

# add_library(localizer SHARED
#         ${HEADERS}
#         ${SOURCES}
#         )

# target_link_libraries(localizer ${OpenCV_LIBS} ${TORCH_LIBRARIES})

add_executable(${PROJECT_NAME} test/junbot_relocalization_node.cpp)
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES})

add_executable(relocalization_test test/relocalization_test.cpp)
target_link_libraries(relocalization_test
        ${catkin_LIBRARIES})

# add_executable(sp_extractor_test test/sp_extractor_test.cpp)
# target_link_libraries(sp_extractor_test
#         localizer
#         ${OpenCV_LIBS}
#         ${TORCH_LIBRARIES}
#         ${catkin_LIBRARIES}
#         )

install(DIRECTORY
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS relocalization_test
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# install(TARGETS sp_extractor_test
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

