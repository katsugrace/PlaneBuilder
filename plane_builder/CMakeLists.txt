cmake_minimum_required(VERSION 3.8)
project(plane_builder)

file(GLOB_RECURSE SOURCES src/*.cpp)

set(PACKAGE_DEPENDS
  rclcpp
)

find_package(ament_cmake REQUIRED)
foreach(package IN ITEMS ${PACKAGE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

include_directories(include)

add_executable(plane_builder src/main.cpp ${SOURCES})

ament_target_dependencies(plane_builder ${PACKAGE_DEPENDS})

install(TARGETS plane_builder
  DESTINATION lib/${PROJECT_NAME})

ament_package()
