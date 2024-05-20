# -*- coding: utf-8 -*-
#  depthai_ros_my/launch/mobilenet_publisher.launch.py
#
# org
# colcon_ws/src/depthai-ros/depthai_examples/launch/mobile_publisher.launch.py
#
# 1. build on SBC and PC
#  $ colcon build --symlink-install --parallel-workers 1 --packages-select depthai_ros_my
#  $ . install/setup.bash
#
# 2. run
# $ ros2 launch depthai_ros_my mobilenet_publisher.launch.py
#

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, launch_description_sources
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
import launch_ros.descriptions
from launch_ros.parameter_descriptions import ParameterValue 

def generate_launch_description():
    depthai_ros_my_path = get_package_share_directory('depthai_ros_my')
    default_resources_path = os.path.join(depthai_ros_my_path,
                                'resources')
                                
    tf_prefix        = LaunchConfiguration('tf_prefix',   default = 'oak')
    base_frame       = LaunchConfiguration('base_frame',    default = 'oak-d_frame')
    parent_frame     = LaunchConfiguration('parent_frame',  default = 'oak-d-base-frame')

    sync_nn            = LaunchConfiguration('sync_nn', default = True)
    nnName            = LaunchConfiguration('nnName', default = "x")
    resourceBaseFolder = LaunchConfiguration('resourceBaseFolder', default = default_resources_path)

    rate     = LaunchConfiguration('rate',       default = 30)          # original 30
    queue_size = LaunchConfiguration('queue_size',       default = 2)   # original 30


    mobilenet_node = launch_ros.actions.Node(
            package='depthai_ros_my', executable='mobilenet_publisher',
            output='screen',
            parameters=[{'tf_prefix': tf_prefix},
                        {'sync_nn': sync_nn},
                        {'nnName': nnName},
                        {'rate': rate},
                        {'queue_size': queue_size},
                        {'resourceBaseFolder': resourceBaseFolder}])


    ld = LaunchDescription()
    
    ld.add_action(mobilenet_node)
    return ld

