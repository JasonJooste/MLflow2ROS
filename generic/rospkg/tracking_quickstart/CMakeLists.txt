cmake_minimum_required(VERSION 3.16.3)

project(tracking_quickstart)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_package()

catkin_install_python(
  PROGRAMS scripts/tracking_quickstart
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
