cmake_minimum_required(VERSION 2.8.3)
project(imu)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs message_generation)
add_service_files(
   FILES
   imu.srv
)
generate_messages(
   DEPENDENCIES
   sensor_msgs
   std_msgs
)

catkin_package(CATKIN_DEPENDS roscpp message_runtime sensor_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(imu src/imu.cpp )
target_link_libraries(imu ${catkin_LIBRARIES})
target_link_libraries(imu LpSensor)

