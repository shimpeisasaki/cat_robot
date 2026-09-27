cmake_minimum_required(VERSION 3.8)
project(ddsm115_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(motor_control src/motor_control.cpp)
target_compile_features(motor_control PUBLIC cxx_std_17)
target_include_directories(motor_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(check_motor_id tools/check_motor_id.cpp)
target_link_libraries(check_motor_id motor_control)
ament_target_dependencies(check_motor_id rclcpp)

add_executable(check_motor_channels tools/check_motor_channels.cpp)
target_link_libraries(check_motor_channels motor_control)
ament_target_dependencies(check_motor_channels rclcpp)

add_executable(set_motor_id tools/set_motor_id.cpp)
target_link_libraries(set_motor_id motor_control)
ament_target_dependencies(set_motor_id rclcpp)

add_executable(base_driver src/base_driver.cpp)
target_link_libraries(base_driver motor_control)
ament_target_dependencies(base_driver rclcpp geometry_msgs nav_msgs diagnostic_msgs std_msgs std_srvs)
target_compile_features(base_driver PUBLIC cxx_std_17)

add_executable(curvature_teleop src/curvature_teleop.cpp)
ament_target_dependencies(curvature_teleop geometry_msgs rclcpp sensor_msgs)
target_compile_features(curvature_teleop PUBLIC cxx_std_17)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS
  motor_control
  check_motor_id
  check_motor_channels
  set_motor_id
  base_driver
  curvature_teleop
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  tools/motor_test_gui
  tools/direct_motor_brake.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)

  ament_cpplint(FILTERS "-legal/copyright")
  ament_flake8(launch tools)
  ament_lint_cmake()
  ament_uncrustify()

  ament_add_gtest(test_differential_drive test/test_differential_drive.cpp)
  target_include_directories(test_differential_drive PRIVATE include)
  ament_add_gtest(test_safety_lease test/test_safety_lease.cpp)
  target_include_directories(test_safety_lease PRIVATE include)
  target_compile_features(test_safety_lease PUBLIC cxx_std_17)
endif()

ament_package()
