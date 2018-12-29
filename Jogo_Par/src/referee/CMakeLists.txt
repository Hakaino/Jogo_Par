cmake_minimum_required(VERSION 2.8.3)
project(referee)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  image_transport
  geometry_msgs
)
catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(referee src/referee.cpp)
target_link_libraries(referee
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  )
