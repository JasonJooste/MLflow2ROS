cmake_minimum_required(VERSION 3.16.3)

project(logreg)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_package()

catkin_install_python(
  PROGRAMS scripts/logreg
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
