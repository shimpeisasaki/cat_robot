cmake_minimum_required(VERSION 3.8)
project(cat_terrain)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(terrain_grid src/terrain_grid.cpp)
target_include_directories(terrain_grid PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(terrain_step_detector src/terrain_step_detector.cpp)
target_link_libraries(terrain_step_detector terrain_grid)
ament_target_dependencies(terrain_step_detector
  rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS terrain_grid terrain_step_detector
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_terrain_grid test/test_terrain_grid.cpp)
  target_link_libraries(test_terrain_grid terrain_grid)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_terrain_parameters test/test_terrain_parameters.py)
endif()

ament_package()
