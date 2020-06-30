cmake_minimum_required(VERSION 2.8.3)
project(jsk_path_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roseus
  std_msgs
  safe_footstep_planner
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/CalcHeightmapGradient.cfg
)
