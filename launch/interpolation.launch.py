import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 경로
    pkg_share = get_package_share_directory('filc')
    config_file = os.path.join(pkg_share, 'config', 'interpolation_config.yaml')
    
    # Launch 인자
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to interpolation config file'
    )
    
    scale_factor_arg = DeclareLaunchArgument(
        'scale_factor',
        default_value='4.0',
        description='Interpolation scale factor (2.0, 3.0, 4.0)'
    )
    
    # Improved Interpolation Node
    interpolation_node = Node(
        package='filc',
        executable='improved_interpolation_node',
        name='improved_interpolation_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'scale_factor': LaunchConfiguration('scale_factor'),
            }
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        scale_factor_arg,
        interpolation_node,
    ])