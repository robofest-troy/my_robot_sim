cmake_minimum_required(VERSION 3.8)
project(my_robot_sim)

find_package(ament_cmake REQUIRED)

# Install all resource directories
install(DIRECTORY
  urdf
  launch
  worlds
  config
  maps
  rviz
  scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
