# -*- coding: utf-8 -*-
# depthai_ros_my/launch/oak-d_rgb_stereo_node.launch.py
#
# 1. build on SBC and PC
#  $ colcon build --symlink-install --parallel-workers 1 --packages-select depthai_ros_my
#  $ . install/setup.bash
#
# 2. run
#      SBC /dev/video0
#      PC  /dev/video1
# $ sudo chmod 777 /dev/video0
# $ ros2 launch depthai_ros_my oak-d_rgb_stereo_node.launch.py
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
    tf_prefix  = LaunchConfiguration('tf_prefix',   default = 'oak')
    mode           = LaunchConfiguration('mode', default = 'depth')
    lrcheck        = LaunchConfiguration('lrcheck', default = True)
    extended       = LaunchConfiguration('extended', default = False)
    subpixel       = LaunchConfiguration('subpixel', default = True)
    confidence     = LaunchConfiguration('confidence', default = 200)
    LRchecktresh   = LaunchConfiguration('LRchecktresh', default = 5)
    #monoResolution = LaunchConfiguration('monoResolution',  default = '720p')
    monoResolution = LaunchConfiguration('monoResolution',  default = '480p')   # OAK-D Lite
    #monoResolution = LaunchConfiguration('monoResolution',  default = '240p')   # OAK-D Lite

    #colorResolution = LaunchConfiguration('colorResolution', default = "1080p")
    colorResolution = LaunchConfiguration('colorResolution', default = "480p")  # OAK-D Lite
    #colorResolution = LaunchConfiguration('colorResolution', default = "240p")  # OAK-D Lite
    useVideo        = LaunchConfiguration('useVideo',        default = True)
    usePreview      = LaunchConfiguration('usePreview',      default = False)
    useDepth        = LaunchConfiguration('useDepth',        default = True)
    previewWidth    = LaunchConfiguration('previewWidth',    default = 300)
    previewHeight   = LaunchConfiguration('previewHeight',   default = 300)

    # IR Brightness. OAK-D-Pro only.
    dotProjectormA   = LaunchConfiguration('dotProjectormA',     default = 0.0)
    floodLightmA     = LaunchConfiguration('floodLightmA',       default = 0.0)
    rate     = LaunchConfiguration('rate',       default = 15)          # original 30
    queue_size = LaunchConfiguration('queue_size',       default = 2)   # original 30
    trace        = LaunchConfiguration('trace', default = True)
    rgb2grey        = LaunchConfiguration('rgb2grey', default = False)
    auto_exp       = LaunchConfiguration('auto_exp', default = False)
    sensIso        = LaunchConfiguration('sensIso', default = 1000)
    expTime        = LaunchConfiguration('expTime', default = 27500)

    oak_parameters={
        'tf_prefix': tf_prefix,
        'mode': mode,
        'lrcheck': lrcheck,
        'extended': extended,
        'subpixel': subpixel,
        'confidence': confidence,
        'LRchecktresh': LRchecktresh,
        'monoResolution': monoResolution,
    }


    oak_parameters_rgb={
        'tf_prefix': tf_prefix,
        'lrcheck': lrcheck,
        'extended': extended,
        'subpixel': subpixel,
        'confidence': confidence,
        'LRchecktresh': LRchecktresh,
        'colorResolution': colorResolution,
        'monoResolution': monoResolution,
        'useVideo': useVideo,
        'usePreview': usePreview,
        'useDepth': useDepth,
        'previewWidth': previewWidth,
        'previewHeight': previewHeight,
        'dotProjectormA': dotProjectormA,
        'floodLightmA': floodLightmA,
        'rate': rate,
        'queue_size': queue_size,
        'trace': trace,
        'rgb2grey': rgb2grey,
        'auto_exp': auto_exp,
        'sensIso': sensIso,
        'expTime': expTime,
    }

    remappings=[
		  ('disparity/image','disparity'),
		  ('disparity/camera_info','right/camera_info'),
		  ('cloud','cloudXYZ')]

    return LaunchDescription([

        DeclareLaunchArgument('queue_size', default_value='2', description=''),
        DeclareLaunchArgument('rate', default_value='30', description=''),

        Node(
            #package='depthai_examples',
            package='depthai_ros_my',
            #executable='rgb_stereo_node',
            executable='rgb_stereo_node_my',
            #name='rgb_stereo_node',
            name='rgb_stereo_node_my',
            output="screen",
            parameters=[oak_parameters_rgb],
            # publish
            # /color/video/camera_info
            # /color/video/image
            # /color/video/image/compressed
            # /color/video/image/compressedDepth
            # /color/video/image/theora
            # /stereo/camera_info
            # /stereo/depth
            # /stereo/depth/compressed
            # /stereo/depth/compressedDepth
            # /stereo/depth/theora
       ),
    ])

