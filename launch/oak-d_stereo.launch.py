# -*- coding: utf-8 -*-
# depthai_ros_my/launch/oak-d_stereo.launch.py
#
# 1. build on SBC and PC
#  $ colcon build --symlink-install --parallel-workers 1 --packages-select depthai_ros_my
#  $ . install/setup.bash
#
# 2. run
#      SBC /dev/video0
#      PC  /dev/video1
# $ sudo chmod 777 /dev/video0
# $ ros2 launch depthai_ros_my oak-d_stereo.launch.py mode:=disparity
#
import os

#from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

'''
TF
 oak
   oak-d-base-frame
     oak-d_frame
       oak_right_camera_optical_frame
'''

def generate_launch_description():
    #tf_prefix  = LaunchConfiguration('tf_prefix',   default = 'oak')
    tf_prefix  = LaunchConfiguration('tf_prefix')
    mode           = LaunchConfiguration('mode', default = 'depth')
    lrcheck        = LaunchConfiguration('lrcheck', default = True)
    extended       = LaunchConfiguration('extended', default = False)
    subpixel       = LaunchConfiguration('subpixel', default = True)
    confidence     = LaunchConfiguration('confidence', default = 200)
    LRchecktresh   = LaunchConfiguration('LRchecktresh', default = 5)
    #monoResolution = LaunchConfiguration('monoResolution',  default = '720p')
    monoResolution = LaunchConfiguration('monoResolution',  default = '480p')   # OAK-D Lite

    rate     = LaunchConfiguration('rate',       default = 15)          # original 30
    queue_size = LaunchConfiguration('queue_size',       default = 2)   # original 30
    trace        = LaunchConfiguration('trace', default = True)
    auto_exp       = LaunchConfiguration('auto_exp', default = False)
    sensIso        = LaunchConfiguration('sensIso', default = 800)
    expTime        = LaunchConfiguration('expTime', default = 20000)

    oak_parameters={
        'tf_prefix': tf_prefix,
        'mode': mode,
        'lrcheck': lrcheck,
        'extended': extended,
        'subpixel': subpixel,
        'confidence': confidence,
        'LRchecktresh': LRchecktresh,
        'monoResolution': monoResolution,
        'rate': rate,
        'queue_size': queue_size,
        'trace': trace,
        'auto_exp': auto_exp,
        'sensIso': sensIso,
        'expTime': expTime,
    }

    remappings=[
		  ('disparity/image','disparity'),
		  ('disparity/camera_info','right/camera_info'),
		  ('cloud','cloudXYZ')]

    return LaunchDescription([

        DeclareLaunchArgument('tf_prefix',default_value='oak', description='tf_prefix'),
        DeclareLaunchArgument('queue_size', default_value='30', description=''),
        DeclareLaunchArgument('rate', default_value='30', description=''),
        DeclareLaunchArgument('mode', default_value='depth', description=''),          # 'depth' or 'disparity'

        Node(
            #package='depthai_examples',
            package='depthai_ros_my',
            #executable='stereo_publisher',
            executable='stereo_publisher_my',
            #name='stereo_publisher',
            name='stereo_publisher_my',
            output="screen",
            parameters=[oak_parameters],
            # publish
            #  left/camera_info
            #  left/image_rect
            #  left/image_rect/compressed
            #  left/image_rect/compressedDepth
            #  left/image_rect/theora
            #  ------
            #  right/camera_info
            #  right/image_rect
            #  right/image_rect/compressed
            #  right/image_rect/compressedDepth
            #  right/image_rect/theora
            #
            #  ----- depth only
            #  stereo/camera_info
            #  stereo/depth
            #  stereo/depth/compressed
            #  stereo/depth/compressedDepth
            #  stereo/depth/theora
            #
            # -----  disparity only
            #  stereo/disparity
            #
        
        ),
    ])

