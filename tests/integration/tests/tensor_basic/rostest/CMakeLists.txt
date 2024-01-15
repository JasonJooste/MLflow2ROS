cmake_minimum_required(VERSION 3.16.3)

project(test_tensor_basic)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_package()

catkin_install_python(
  PROGRAMS test/test_tensor_basic
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/test_tensor_basic.test)
endif()