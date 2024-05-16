# -*- coding: utf-8 -*-
# rtabmap_ros_my/launch/oak-d_rgb_stereo_test1.launch.py
#
# 1. build on SBC and PC
#  $ colcon build --symlink-install --parallel-workers 1 --packages-select depthai_ros_my
#  $ . install/setup.bash
#
# 2. run
#      SBC /dev/video0
#      PC  /dev/video1
# $ sudo chmod 777 /dev/video0
# $ ros2 launch depthai_ros_my oak-d_rgb_stereo_test1.launch.py
#
import os

#from launch import LaunchDescription
from launch_ros.actions import Node



from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo, OpaqueFunction

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch.conditions import IfCondition, UnlessCondition

'''
TF
 oak
   - oak-d-base-frame
     - oak-d_frame
        - oak_right_camera_optical_frame
'''

def generate_launch_description():
    depthai_examples = get_package_share_directory('depthai_examples')
    #stereo_image_proc = get_package_share_directory('stereo_image_proc')
    depthai_ros_my = get_package_share_directory('depthai_ros_my')

    parameters={
        'voxel_size':0.05,
        'decimation7':1,
        "max_depth":4
    }

    remappings=[
		  ('disparity/image','disparity'),
		  ('disparity/camera_info','right/camera_info'),
		  ('cloud','cloudXYZ')]

    return LaunchDescription([

        DeclareLaunchArgument('cloud',default_value='true', description='Point cloud (optional).'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
            output="screen",
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_base_link',
            #arguments=['0', '0', '0.2', '-1.5707963267948966', '0', '-1.5707963267948966', 'base_link', 'stereo_camera'],
            arguments=['0', '0', '1.0', '0', '0', '0', 'base_link', 'oak'],
            output="screen",
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_base_link2',
            #arguments=['0', '0', '0', '0', '0', '0', 'oak', 'oak-d-base-frame'],
            arguments=['0', '0', '0', '-1.5707963267948966', '0', '-1.5707963267948966', 'oak', 'oak-d-base-frame'],
            output="screen",
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_base_link3',
            arguments=['0', '0', '0', '0', '0', '0', 'oak-d-base-frame', 'oak-d_frame'],
            output="screen",
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_base_center',
            arguments=['0', '0', '0', '0', '0', '0', 'oak-d_frame', 'oak_rgb_camera_optical_frame'],
            output="screen",
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_base_right',
            arguments=['0.0375', '0', '0', '0', '0', '0', 'oak-d_frame', 'oak_right_camera_optical_frame'],
            output="screen",
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_base_left',
            arguments=['-0.0375', '0', '0', '0', '0', '0', 'oak-d_frame', 'oak_left_camera_optical_frame'],
            output="screen",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_ros_my , 'launch', 'oak-d_rgb_stereo_node.launch.py')
            ),
            #launch_arguments={'camera_model': 'OAK-D-LITE'}.items(),
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

        Node(
            # http://wiki.ros.org/rtabmap_ros#rtabmap_ros.2Fpoint_cloud_xyz
            # https://github.com/ros-perception/image_pipeline/tree/foxy/depth_image_proc/src
            # camera_info (sensor_msgs/CameraInfo) 
            # image_rect (sensor_msgs/Image) 
            package='rtabmap_util', executable='point_cloud_xyz', output='screen',
            condition=IfCondition(LaunchConfiguration("cloud")),
            parameters=[{
                "decimation": 4,
                #"voxel_size": 0.0,
                "voxel_size": 0.05,
                "approx_sync": True,
                #"exact_sync": True,
                #"approx_sync_max_interval": 0.1 ,
                #"approx_sync_max_interval": 0.2 ,
                "approx_sync_max_interval": 0.5 ,
                #"approx_sync_max_interval": 0.7 ,
                "qos": 1,
            }],
            remappings=[
                #('disparity/image', '/disparity'),   #
                #('disparity/camera_info', '/right/camera_info'),
                ('depth/camera_info','/stereo/camera_info'),
                ('depth/image','/stereo/depth'),
                ('cloud', '/cloudXYZ')],
            #namespace=LaunchConfiguration('namespace'),
            # subscribe
            #  depth/camera_info
            #  depth/image
            #  ------
            #  disparity/camera_info
            #  disparity/image
            # publish
            #  cloud
        ),
   ])

