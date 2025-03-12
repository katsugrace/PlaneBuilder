#!/usr/bin/env python3

from launch import LaunchDescription
from src.nodes import plane_builder_node


def generate_launch_description():
    return LaunchDescription([
        plane_builder_node()
    ])
