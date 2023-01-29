cmake_minimum_required(VERSION 2.8)
project(orb_slam3_ros)

IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF ()

set(CMAKE_CXX_STANDARD 14)

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# Check C++14 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if (COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++14.")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif ()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4.2)
if (NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV 4.2 not found.")
endif ()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        image_transport
        geometry_msgs
        sensor_msgs
        nav_msgs
        std_msgs
        message_filters
        roscpp
        rospy
        tf
        tf2
        message_generation
        )

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

find_package(PCL 1.10 REQUIRED)
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
message("PCL Include Directory: ${PCL_INCLUDE_DIRS}")

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})


find_package(Boost COMPONENTS system filesystem serialization REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

list(APPEND CMAKE_PREFIX_PATH "~/Github/libtorch/")
find_package(Torch REQUIRED)
include_directories(${TORCH_INCLUDE_DIRS})

# cuda
find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)

# tensorrt
include_directories(/usr/include/x86_64-linux-gnu/)
link_directories(/usr/lib/x86_64-linux-gnu/)

add_service_files(
        FILES
        SaveMap.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge image_transport tf sensor_msgs dynamic_reconfigure message_runtime
        LIBRARIES {PROJECT_NAME} libDBoW2 libg2o libyolov5_detector
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/orb_slam3
        ${PROJECT_SOURCE_DIR}/orb_slam3/include
        ${PROJECT_SOURCE_DIR}/orb_slam3/include/CameraModels
        ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty
        ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/Sophus
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

include(${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2/CMakeLists.txt)
include(${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o/CMakeLists.txt)
#include(${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/yolov5/CMakeLists.txt)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/orb_slam3/lib)

add_library(${PROJECT_NAME} SHARED
        orb_slam3/src/System.cc
        orb_slam3/src/Tracking.cc
        orb_slam3/src/LocalMapping.cc
        orb_slam3/src/LoopClosing.cc
        orb_slam3/src/ORBextractor.cc
        orb_slam3/src/ORBmatcher.cc
        orb_slam3/src/FrameDrawer.cc
        orb_slam3/src/Converter.cc
        orb_slam3/src/MapPoint.cc
        orb_slam3/src/KeyFrame.cc
        orb_slam3/src/Atlas.cc
        orb_slam3/src/Map.cc
        orb_slam3/src/MapDrawer.cc
        orb_slam3/src/Optimizer.cc
        orb_slam3/src/Frame.cc
        orb_slam3/src/KeyFrameDatabase.cc
        orb_slam3/src/Sim3Solver.cc
        orb_slam3/src/Viewer.cc
        orb_slam3/src/ImuTypes.cc
        orb_slam3/src/G2oTypes.cc
        orb_slam3/src/CameraModels/Pinhole.cpp
        orb_slam3/src/CameraModels/KannalaBrandt8.cpp
        orb_slam3/src/OptimizableTypes.cpp
        orb_slam3/src/MLPnPsolver.cpp
        orb_slam3/src/GeometricTools.cc
        orb_slam3/src/TwoViewReconstruction.cc
        orb_slam3/src/PointCloudMapping.cc
        orb_slam3/src/YoloDetection.cpp
        orb_slam3/src/Flow.cpp
        orb_slam3/src/ObjectDatabase.cpp
        orb_slam3/src/Config.cc
        orb_slam3/src/Settings.cc
        orb_slam3/include/System.h
        orb_slam3/include/Tracking.h
        orb_slam3/include/LocalMapping.h
        orb_slam3/include/LoopClosing.h
        orb_slam3/include/ORBextractor.h
        orb_slam3/include/ORBmatcher.h
        orb_slam3/include/FrameDrawer.h
        orb_slam3/include/Converter.h
        orb_slam3/include/MapPoint.h
        orb_slam3/include/KeyFrame.h
        orb_slam3/include/Atlas.h
        orb_slam3/include/Map.h
        orb_slam3/include/MapDrawer.h
        orb_slam3/include/Optimizer.h
        orb_slam3/include/Frame.h
        orb_slam3/include/KeyFrameDatabase.h
        orb_slam3/include/Sim3Solver.h
        orb_slam3/include/Viewer.h
        orb_slam3/include/ImuTypes.h
        orb_slam3/include/G2oTypes.h
        orb_slam3/include/CameraModels/GeometricCamera.h
        orb_slam3/include/CameraModels/Pinhole.h
        orb_slam3/include/CameraModels/KannalaBrandt8.h
        orb_slam3/include/OptimizableTypes.h
        orb_slam3/include/MLPnPsolver.h
        orb_slam3/include/GeometricTools.h
        orb_slam3/include/TwoViewReconstruction.h
        orb_slam3/include/SerializationUtils.h
        orb_slam3/include/Config.h
        orb_slam3/include/Settings.h
        orb_slam3/include/PointCloudMapping.h
        orb_slam3/include/YoloDetection.h
        orb_slam3/include/Flow.h
        orb_slam3/include/ObjectDatabase.h)

SET(TRT_LIBS nvinfer)

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OCTOMAP_LIBRARIES}
        ${TORCH_LIBRARIES}
        ${TRT_LIBS}
        ${CUDA_CUDA_LIBRARY}
        ${CUDA_CUDART_LIBRARY}
        ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o/lib/libg2o.so
        #  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/yolov5/lib/libyolov5_detector.so
        -lboost_serialization
        -lcrypto
        )

## ROS node
add_executable(simple_rgbd_node
        src/simple_rgbd_node.cpp
        )
target_link_libraries(simple_rgbd_node
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )
