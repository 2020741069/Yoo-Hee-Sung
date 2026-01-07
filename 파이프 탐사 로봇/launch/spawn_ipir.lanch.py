import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    share_dir = get_package_share_directory('inpipe_robot')
    decalre_robot_description = DeclareLaunchArgument('robot_description')

    x_pose = LaunchConfiguration('x_pose', default=0.0)
    y_pose = LaunchConfiguration('y_pose', default=0.0)
    z_pose = LaunchConfiguration('z_pose', default=0.0)
    declare_x_pose_cmd = DeclareLaunchArgument('x_pose', default_value='0.0')
    declare_y_pose_cmd = DeclareLaunchArgument('y_pose', default_value='0.0')
    declare_z_pose_cmd = DeclareLaunchArgument('z_pose', default_value='0.0')

    gz_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'ipir',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-R', '3.14159',
        ],
        output='screen'
    )

    params = os.path.join(
        share_dir, 'config', 'ipir_bridge.yaml'
    )

    gz_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={params}'
        ],
        output='screen'
    )

    gz_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen'
    )

    return LaunchDescription([
        gz_bridge_cmd,
        decalre_robot_description,
        declare_x_pose_cmd,
        declare_y_pose_cmd,
        declare_z_pose_cmd,
        gz_spawner_cmd,
        gz_image_bridge_cmd,
    ])
