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

catkin_package(
  CATKIN_DEPENDS 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(image_detect 
  src/image_detect.cpp
)
target_link_libraries(image_detect
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(mouse 
  src/mouse.cpp
)
target_link_libraries(mouse
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
