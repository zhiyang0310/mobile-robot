cmake_minimum_required(VERSION 2.8.3)
project(odom)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs geometry_msgs message_generation encoders imu tf)
find_package(Boost 1.58.0 REQUIRED COMPONENTS system filesystem)

catkin_package(CATKIN_DEPENDS roscpp message_runtime nav_msgs sensor_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odom src/odom.cpp)
add_executable(odom_encoder_only src/odom_encoder_only.cpp)

target_link_libraries(odom ${catkin_LIBRARIES})
target_link_libraries(odom ${Boost_LIBRARIES})
target_link_libraries(odom_encoder_only ${catkin_LIBRARIES})
target_link_libraries(odom_encoder_only ${Boost_LIBRARIES})

