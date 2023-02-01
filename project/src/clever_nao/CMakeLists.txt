cmake_minimum_required(VERSION 2.8.3)
project(clever_nao)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
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
    FaceDetect.srv
    Hear.srv
    QRCode.srv
    Speak.srv
    Walk.srv
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(CATKIN_DEPENDS message_runtime)

add_library(ticket src/ticket_checker.cpp)


#make python file executable
catkin_install_python(PROGRAMS ./scripts/face_server.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS ./scripts/hear_server.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS ./scripts/speak_server.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS ./scripts/walk_server.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(qrcode_server src/qrcode_server.cpp)
target_link_libraries(qrcode_server
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)


# add_executable(main src/main.cpp)
# target_link_libraries(main 
#   ticket
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBS}
#   ${aruco_LIBS}
# )
# add_dependencies(main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
