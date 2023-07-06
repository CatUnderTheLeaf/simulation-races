cmake_minimum_required(VERSION 2.8.3)
project(control_tf_odometry)


find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf
)

catkin_package(
 CATKIN_DEPENDS ackermann_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(control_tf_odometry_node src/rbcar_robot_control.cpp)


add_dependencies(control_tf_odometry_node control_tf_odometry_generate_messages_cpp)


target_link_libraries(control_tf_odometry_node
   ${catkin_LIBRARIES}
)