cmake_minimum_required(VERSION 3.8)
project(robot_mtc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(robot_interfaces REQUIRED)

add_executable(pick_place_mtc src/pick_place_mtc.cpp)
target_include_directories(pick_place_mtc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pick_place_mtc PUBLIC cxx_std_17)
ament_target_dependencies(pick_place_mtc
  rclcpp
  moveit_task_constructor_core
  moveit_ros_planning_interface
  geometry_msgs
  tf2_eigen
  robot_interfaces
)

install(TARGETS pick_place_mtc
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
