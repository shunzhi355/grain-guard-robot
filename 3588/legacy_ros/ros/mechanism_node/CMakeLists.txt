cmake_minimum_required(VERSION 3.0.2)
project(mechanism_node)

find_package(catkin REQUIRED COMPONENTS message_generation)

add_service_files(
  FILES
  SetGrain.srv
  MoveLift.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime
)
