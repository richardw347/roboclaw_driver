cmake_minimum_required(VERSION 2.8.3)
project(roboclaw_driver)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp diagnostic_msgs geometry_msgs nav_msgs tf)

## Declare a catkin package
catkin_package()

set(SOURCE_FILES
    ${PROJECT_SOURCE_DIR}/src/Roboclaw.h
    ${PROJECT_SOURCE_DIR}/src/Roboclaw.cpp
    ${PROJECT_SOURCE_DIR}/src/RoboclawNode.cpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(roboclaw_driver ${SOURCE_FILES})
target_link_libraries(roboclaw_driver ${catkin_LIBRARIES})
