cmake_minimum_required(VERSION 2.8.3)
project(player3)
find_package(catkin REQUIRED roscpp)
catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(P_3 src/P_3.cpp)
target_link_libraries(P_3
  ${catkin_LIBRARIES}
)
