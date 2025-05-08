<p align="center">
  <img src="./resources/logo.png" alt="Project Logo" width="5000"/>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/ROS%202-000000?style=flat&logo=ros" />
  <img src="https://img.shields.io/badge/build-CMake-blue" />
  <img src="https://img.shields.io/badge/C%2B%2B-17-blue" />
  <img src="https://img.shields.io/badge/python-3.12-blue" />
</p>

<p align="center">
  A utility that computes a plane from three given points in 3D space. </br>
  This tool is useful for defining an interaction surface where various operations—such as object placement.
</p>

## ⚙️ Build Instructions

```bash
mkdir -p ~/ros2_ws/src/
cd ~/ros2_ws/src/
git clone https://github.com/katsugrace/PlaneBuilder.git
cd PlaneBuilder
colcon build
source install/setup.bash
```

## 🚀 Use Instructions

```bash
ros2 launch plane_builder_launch plane_builder.launch.py
```
