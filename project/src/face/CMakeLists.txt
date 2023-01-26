cmake_minimum_required(VERSION 3.0.2)
project(face)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  message_generation
  cv_bridge
)

find_package(OpenCV REQUIRED)
# find_package(face_recognition REQUIRED)

# generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
  CATKIN_DEPENDS rospy message_runtime
)

catkin_install_python(PROGRAMS src/face.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

