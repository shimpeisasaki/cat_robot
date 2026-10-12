cmake_minimum_required(VERSION 3.8)
project(cat_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz map DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/navigation_guard scripts/waypoint_route scripts/waypoint_editor
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/waypoint_io.py DESTINATION lib/${PROJECT_NAME})

ament_package()
