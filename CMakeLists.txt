cmake_minimum_required(VERSION 2.8.3)
project(video_writer)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    cv_bridge
    image_transport
)
find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

### EXECUTABLES ###############################################
add_executable(video_writer
  ros/src/video_writer_node.cpp
)
target_link_libraries(video_writer
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
