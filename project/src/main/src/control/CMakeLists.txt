cmake_minimum_required(VERSION 2.8.3)
project(control)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rospy
  message_generation
  sensor_msgs
  cv_bridge
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
    Speak.srv
    Speech.srv
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(CATKIN_DEPENDS message_runtime)


#make python file executable
catkin_install_python(PROGRAMS ./script/speech_server.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS ./script/speak_server.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



add_executable(control src/Application/main.cpp)
add_library(ticket src/ticket_checker.cpp)

target_link_libraries(control 
  ticket
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${aruco_LIBS}
)
add_dependencies(control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
