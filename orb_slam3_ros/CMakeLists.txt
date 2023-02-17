cmake_minimum_required(VERSION 3.2)
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

set(OpenCV_DIR "/home/hn/opencv-4.5.2/build")
find_package(OpenCV 4.5 REQUIRED NO_MODULE PATHS /usr/local NO_DEFAULT_PATH)
if (NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV 4.5 not found.")
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
        vision_msgs
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

find_package(PCL 1.8 REQUIRED)
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
message("PCL Include Directory: ${PCL_INCLUDE_DIRS}")

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})

find_package(Boost COMPONENTS system filesystem serialization REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

# list(APPEND CMAKE_PREFIX_PATH "~/Github/libtorch/")
# find_package(Torch REQUIRED)
# include_directories(${TORCH_INCLUDE_DIRS})

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
        LIBRARIES {PROJECT_NAME} libDBoW2 libg2o libyolov5-detector
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/orb_slam3
        ${PROJECT_SOURCE_DIR}/orb_slam3/include
        ${PROJECT_SOURCE_DIR}/orb_slam3/include/CameraModels
        ${PROJECT_SOURCE_DIR}/orb_slam3/include/cuda
        ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty
        ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/Sophus
        ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/yolov5
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

include(${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2/CMakeLists.txt)
include(${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o/CMakeLists.txt)
# include(${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/yolov5/CMakeLists.txt)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/orb_slam3/lib)

file(GLOB HEADERS 
        ${CMAKE_CURRENT_SOURCE_DIR}/orb_slam3/include/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/orb_slam3/include/*.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/orb_slam3/include/CameraModels/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/orb_slam3/include/cuda/*.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/orb_slam3/include/line_lbd/*.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/orb_slam3/include/line_lbd/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/orb_slam3/include/line_lbd/line_descriptor/*.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/orb_slam3/include/detect_3d_cuboid/*.h
        )

file(GLOB SOURCES 
        ${CMAKE_CURRENT_SOURCE_DIR}/orb_slam3/src/*.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/orb_slam3/src/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/orb_slam3/src/CameraModels/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/orb_slam3/src/cuda/*.cu
        ${CMAKE_CURRENT_SOURCE_DIR}/orb_slam3/src/line_detect/libs/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/orb_slam3/src/line_detect/libs/*.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/orb_slam3/src/line_detect/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/orb_slam3/src/detect_3d_cuboid/*.cpp
        )

source_group("Header Files" FILES ${HEADERS})
source_group("Source Files" FILES ${SOURCES})

cuda_add_library(${PROJECT_NAME} SHARED
        ${HEADERS}
        ${SOURCES}
        )

 set(TRT_LIBS nvinfer)

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OCTOMAP_LIBRARIES}
        # ${TORCH_LIBRARIES}
        ${TRT_LIBS}
        ${CUDA_CUDA_LIBRARY}
        ${CUDA_CUDART_LIBRARY}
        ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o/lib/libg2o.so
        ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/yolov5/lib/libyolov5-detector.so
        -lboost_serialization
        -lcrypto
         nvToolsExt
#        opencv_cudabgsegm
#        opencv_cudaobjdetect
#        opencv_cudastereo
#        opencv_cudafeatures2d
#        opencv_cudacodec
#        opencv_cudaoptflow
#        opencv_cudalegacy
#        opencv_cudawarping
#        opencv_cudaimgproc
#        opencv_cudafilters
        )

## ROS node
add_executable(simple_rgbd_node
        src/simple_rgbd_node.cpp)
target_link_libraries(simple_rgbd_node
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

## ZED ROS node
add_executable(zed_rgbd_node
        src/zed_node_rgbd.cpp
        src/common.cc)
target_link_libraries(zed_rgbd_node
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

## D435i ROS node
add_executable(d435i_rgbd_node
        src/d435i_rgbd.cpp
        src/common.cc)
target_link_libraries(d435i_rgbd_node
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )
