cmake_minimum_required(VERSION 2.8.3)
project(tutorial_5_ws)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy
  message_generation
  sensor_msgs
  cv_bridge
  aruco
  aruco_msgs
  image_transport
)

find_package(OpenCV REQUIRED)

add_service_files(
    DIRECTORY srv
    FILES
    MoveJoints.srv
)

find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN_DEFINATIONS})

generate_messages(DEPENDENCIES std_msgs)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package(CATKIN_DEPENDS message_runtime)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#make python file executable
catkin_install_python(PROGRAMS ./script/move_service.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



add_executable(nao_control_tutorial_1 src/client.cpp)
target_link_libraries(nao_control_tutorial_1 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${aruco_LIBS}
)
add_dependencies(nao_control_tutorial_1 ${catkin_EXPORTED_TARGETS})
