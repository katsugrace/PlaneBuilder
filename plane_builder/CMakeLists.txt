cmake_minimum_required(VERSION 3.8)
project(plane_builder)

set(PACKAGE_DEPENDS
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  Eigen3
  plane_builder_msgs
)

find_package(ament_cmake REQUIRED)
foreach(package IN ITEMS ${PACKAGE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

file(GLOB_RECURSE SOURCES src/*.cpp)

include_directories(include)

add_executable(plane_builder src/main.cpp ${SOURCES})
target_link_libraries(plane_builder Eigen3::Eigen)

ament_target_dependencies(plane_builder ${PACKAGE_DEPENDS})

install(TARGETS plane_builder
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION
  share/${PROJECT_NAME})

ament_package()
