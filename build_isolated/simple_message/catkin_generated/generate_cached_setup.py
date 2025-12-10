# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/noetic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/noetic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in '/home/ROS/workspace/devel_isolated/ros_tcp_endpoint;/home/ROS/workspace/devel_isolated/hiwin_driver;/home/ROS/workspace/devel_isolated/pass_through_controllers;/home/ROS/workspace/devel_isolated/my_hiwin_pkg;/home/ROS/workspace/devel_isolated/industrial_utils;/home/ROS/workspace/devel_isolated/industrial_trajectory_filters;/home/ROS/workspace/devel_isolated/industrial_robot_status_controller;/home/ROS/workspace/devel_isolated/industrial_robot_status_interface;/home/ROS/workspace/devel_isolated/industrial_robot_simulator;/home/ROS/workspace/devel_isolated/industrial_msgs;/home/ROS/workspace/devel_isolated/industrial_deprecated;/home/ROS/workspace/devel_isolated/industrial_core;/home/ROS/workspace/devel_isolated/hiwin_ra610_1476_moveit_config;/home/ROS/workspace/devel_isolated/hiwin_description;/home/ROS/workspace/devel_isolated/custom_msgs;/home/ROS/workspace/devel;/opt/ros/noetic'.split(';'):
        python_path = os.path.join(workspace, 'lib/python3/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/ROS/workspace/devel_isolated/simple_message/env.sh')

output_filename = '/home/ROS/workspace/build_isolated/simple_message/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    # print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
