cmake_minimum_required(VERSION 3.8)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(
  plane_builder_tests
  plane_builder.cpp
)