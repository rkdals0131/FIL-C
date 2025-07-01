import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 파라미터 선언
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/ouster/points',
        description='Input point cloud topic'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/ouster/interpolated_points',
        description='Output interpolated point cloud topic'
    )
    
    scale_factor_arg = DeclareLaunchArgument(
        'scale_factor',
        default_value='4.0',
        description='Interpolation scale factor (e.g., 4.0 for 32->128 channels)'
    )
    
    # 테스트 보간 노드
    test_interpolation_node = Node(
        package='filc',
        executable='test_interpolation_node',
        name='test_interpolation_node',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'scale_factor': LaunchConfiguration('scale_factor'),
        }],
        remappings=[
            ('input_cloud', LaunchConfiguration('input_topic')),
            ('output_cloud', LaunchConfiguration('output_topic')),
        ]
    )
    
    # 시각화 노드 (Python)
    visualizer_node = Node(
        package='filc',
        executable='visualize_interpolation.py',
        name='interpolation_visualizer',
        output='screen',
        parameters=[{
            'original_topic': LaunchConfiguration('input_topic'),
            'interpolated_topic': LaunchConfiguration('output_topic'),
        }]
    )
    
    return LaunchDescription([
        input_topic_arg,
        output_topic_arg,
        scale_factor_arg,
        test_interpolation_node,
        visualizer_node,
    ])