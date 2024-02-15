cmake_minimum_required(VERSION 3.0.2)
project(sklearn_tensor_basic_model_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  sklearn_tensor_basic_model.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 LIBRARIES sklearn_tensor_basic_model_msgs
 CATKIN_DEPENDS std_msgs
CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)