cmake_minimum_required(VERSION 3.0.2)
project(screw_moveit_integration)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/apply_workcell_scene.py
    scripts/moveit_bridge_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
