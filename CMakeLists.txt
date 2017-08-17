cmake_minimum_required(VERSION 2.8.3)
project(project_3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  message_generation
)

find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  Image_info.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(image_detect_stereo
  src/image_detect_stereo.cpp
)
target_link_libraries(image_detect_stereo
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

