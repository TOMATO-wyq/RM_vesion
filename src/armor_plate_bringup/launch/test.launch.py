from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    # 定义参数
    video_default_path = get_package_share_directory("armor_plate_identification") + "/video/Blue.mp4"
    video_true_path1 = get_package_share_directory("armor_plate_identification") + "/video/1.avi"
    video_true_path2 = get_package_share_directory("armor_plate_identification") + "/video/2.avi"
    
    video_path_arg = DeclareLaunchArgument(
        name="video_path",
        default_value= video_default_path 
        # default_value= video_true_path1 
        # default_value= video_true_path2 
    )

    identification_node_params_file = os.path.join(
        get_package_share_directory('armor_plate_identification'),
        'config',
        'params.yaml'
    )
    tracker_node_params_file = os.path.join(
        get_package_share_directory('armor_plate_tracker'),
        'config',
        'params.yaml'
    )
    serial_node_params_file = os.path.join(
        get_package_share_directory('armor_plate_serial'),
        'config',
        'params.yaml'
    )

    foxglove_layout_file = os.path.join(
        get_package_share_directory('armor_plate_bringup'),
        'FoxGlove',
        '装甲板.json'
    )

    use_foxglove_arg = DeclareLaunchArgument(
        'use_foxglove',
        default_value='true',
        description='Whether to start Foxglove Bridge'
    )

    # 定义节点
    test_node = Node(
        package="armor_plate_identification",
        executable="Test",
        arguments=[LaunchConfiguration("video_path")],
        output='screen',
        emulate_tty=True,
        parameters=[identification_node_params_file]
    )

    tracker_node = Node(
        package='armor_plate_tracker',
        executable='armor_plate_tracker_node',
        name='armor_plate_tracker_node',
        output='screen',
        emulate_tty=True,
        parameters=[tracker_node_params_file]
    )

    serial_node = Node(
        package='armor_plate_serial',
        executable='serial_node',
        name='armor_plate_serial_node',
        output='screen',
        emulate_tty=True,
        parameters=[serial_node_params_file]
    )

    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_foxglove')),
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
        }]
    )

    return LaunchDescription([
        # 定义参数
        video_path_arg,
        use_foxglove_arg,
        # 启动节点
        test_node, tracker_node, serial_node, foxglove_bridge_node
    ])
