cmake_minimum_required(VERSION 2.8.3)
project(player1)
find_package(catkin REQUIRED roscpp)
catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(P_1 src/P_1.cpp)
target_link_libraries(P_1
  ${catkin_LIBRARIES}
)
