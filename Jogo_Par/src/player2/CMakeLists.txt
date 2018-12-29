cmake_minimum_required(VERSION 2.8.3)
project(player2)
find_package(catkin REQUIRED roscpp)
catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(P_2 src/P_2.cpp)
target_link_libraries(P_2
  ${catkin_LIBRARIES}
)
