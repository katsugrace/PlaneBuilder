cmake_minimum_required(VERSION 3.8)
project(plane_builder_launch)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(src)

install(DIRECTORY launch DESTINATION
  share/${PROJECT_NAME})

ament_package()
