cmake_minimum_required(VERSION 2.8.3)
project(player4)
find_package(catkin REQUIRED roscpp)
catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(P_4 src/P_4.cpp)
target_link_libraries(P_4
  ${catkin_LIBRARIES}
)
