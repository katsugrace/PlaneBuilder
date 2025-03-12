#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
import yaml


def get_params(pkg_name: str, folder: str = 'config', file_name: str = 'params.yaml') -> dict:
    params_path = os.path.join(get_package_share_directory(pkg_name), folder, file_name)
    if not os.path.exists(params_path):
        return {}

    try:
        with open(params_path, 'r') as file:
            params = yaml.safe_load(file) or {}
    except yaml.YAMLError as ex:
        print(f'YAML error: {str(ex)}')
        return {}
    except Exception as ex:
        print(f'An error occurred: {str(ex)}')
        return {}

    return params
