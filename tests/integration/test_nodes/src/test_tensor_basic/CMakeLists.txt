cmake_minimum_required(VERSION 3.16.3)

project(test_tensor_basic)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_package()

catkin_install_python(
  PROGRAMS scripts/test_tensor_basic
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
