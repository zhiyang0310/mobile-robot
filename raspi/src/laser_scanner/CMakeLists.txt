cmake_minimum_required(VERSION 2.8.3)
project(laser_scanner)
set(DELTA_2B_LIDAR_SDK_PATH "./sdk/")

FILE(GLOB DELTA_2B_LIDAR_SDK_SRC "${DELTA_2B_LIDAR_SDK_PATH}/src/*.cpp")

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs)

catkin_package(CATKIN_DEPENDS roscpp message_runtime std_msgs sensor_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${DELTA_2B_LIDAR_SDK_PATH}/include)

add_executable(laser_scan ./src/laser_scan.cpp ${DELTA_2B_LIDAR_SDK_SRC})
target_link_libraries(laser_scan ${catkin_LIBRARIES})
target_link_libraries(laser_scan -lrt -lpthread)

