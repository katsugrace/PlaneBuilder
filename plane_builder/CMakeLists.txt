cmake_minimum_required(VERSION 3.8)
project(plane_builder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

file(GLOB_RECURSE SOURCES src/*.cpp)

set(dependencies
  rclcpp
)

include_directories(include)

add_executable(${PROJECT_NAME} src/main.cpp ${SOURCES})
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(test)
endif()

ament_package()
