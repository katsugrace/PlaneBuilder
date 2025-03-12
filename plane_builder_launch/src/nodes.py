#!/usr/bin/env python3

from launch_ros.actions import Node
from src.params import get_params


def plane_builder_node():
    params = get_params(pkg_name='plane_builder')
    return Node(
      package='plane_builder',
      executable='plane_builder',
      parameters=[params]
    )
