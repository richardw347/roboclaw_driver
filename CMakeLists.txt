cmake_minimum_required(VERSION 2.8.3)
project(roboclaw_driver)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp diagnostic_msgs geometry_msgs nav_msgs tf)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB1 libusb-1.0 REQUIRED)
include_directories(${LIBUSB1_INCLUDE_DIRS})

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

file(GLOB_RECURSE HEADER_FILES include/*.h)

add_library(roboclaw src/RoboClaw.cpp)
target_link_libraries(roboclaw ${LIBUSB1_LDFLAGS} ${roscpp_LIBRARIES})

set(SOURCE_FILES
    ${PROJECT_SOURCE_DIR}/src/RoboclawNode.cpp
)

add_executable(roboclaw_driver ${SOURCE_FILES})
target_link_libraries(roboclaw_driver ${catkin_LIBRARIES})
target_link_libraries(roboclaw_driver roboclaw)
