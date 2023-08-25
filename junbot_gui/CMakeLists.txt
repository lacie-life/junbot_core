cmake_minimum_required(VERSION 3.0.2)
project(junbot_gui)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_definitions(-std=c++14 -O3)

set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(ROS_DEPENDS
    tf
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
    image_transport
    diagnostic_msgs)

find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDS})

# set(CMAKE_PREFIX_PATH "~/Qt/5.15.2/gcc_64")

set(ModuleList
    Core
    Gui
    Widgets
    Multimedia
    Network
    Sql
    Svg
    Mqtt
)

foreach(QtModule ${ModuleList})
    set(Qt5${QtModule}_DIR ${CMAKE_PREFIX_PATH}/lib/cmake/Qt5${QtModule})
    message("foreach-> Qt5${QtModule}_DIR : ${Qt5${QtModule}_DIR}")

    set(QT_LIBRARIES ${QT_LIBRARIES} Qt5::${QtModule})
endforeach()
message("${QT_LIBRARIES}")

find_package(Qt5 REQUIRED ${ModuleList})

find_package(OpenCV 4.2 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)
file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

source_group("Header Files" FILES ${HEADERS})
source_group("Source Files" FILES ${SOURCES})

set(UI 
    ${CMAKE_CURRENT_LIST_DIR}/ui/loginwidget.ui
    ${CMAKE_CURRENT_LIST_DIR}/ui/robotinterface.ui
    ${CMAKE_CURRENT_LIST_DIR}/ui/addNewTarget.ui
    )

qt5_wrap_ui(UI_HEADERS ${UI})

list(APPEND RESOURCES resources.qrc)

qt5_add_resources(RCC_SOURCES ${RESOURCES})

catkin_package(
    INCLUDE_DIRS include
    #  LIBRARIES JunBotGUI
    CATKIN_DEPENDS ${ROS_DEPENDS}
    #  DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(junbot_gui
    ${HEADERS}
    ${SOURCES}
    ${UI_HEADERS}
    ${RCC_SOURCES}
)

target_link_libraries(junbot_gui PRIVATE
    ${QT_LIBRARIES}
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES})


file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

install(DIRECTORY data/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

