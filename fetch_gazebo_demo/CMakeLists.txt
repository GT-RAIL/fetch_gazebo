cmake_minimum_required(VERSION 2.8.3)
project(fetch_gazebo_demo)

find_package(catkin REQUIRED COMPONENTS
  rospy
)
catkin_python_setup()
catkin_package()

install(
  PROGRAMS
    scripts/demo.py
    scripts/pick_place_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch maps config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
