cmake_minimum_required(VERSION 2.8.3)
project(nao_control_tutorial_2)

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
  tf
  geometry_msgs
)

#find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
#add_definitions(${EIGEN_DEFINATIONS})

add_service_files(
    DIRECTORY srv
    FILES
    MoveJoints.srv
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(CATKIN_DEPENDS message_runtime)


#make python file executable
catkin_install_python(PROGRAMS ./script/move_service.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



add_executable(nao_control_tutorial_2 src/client.cpp)
target_link_libraries(nao_control_tutorial_2 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${aruco_LIBS}
)
add_dependencies(nao_control_tutorial_2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
