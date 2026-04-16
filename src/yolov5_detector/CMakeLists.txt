cmake_minimum_required(VERSION 3.0.2)
project(yolov5_detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_filters
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  visualization_msgs
  cv_bridge
  image_transport
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)



install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/block_detector_yolov5_node.py
  scripts/easyocr_digit_node.py
  scripts/room_digit_detector_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

