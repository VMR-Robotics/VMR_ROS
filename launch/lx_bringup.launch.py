import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. 获取参数文件路径
    default_config_path = os.path.join(
        get_package_share_directory('vmr_ros_pkg'),
        'config',
        'lx_config.yaml'
    )

    # 2. 声明 launch 参数，允许在命令行覆盖
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_path,
            description='Full path to lx_config.yaml'
        ),

        Node(
            package='vmr_ros_pkg',
            executable='lx_ros_interface',
            name='lx_ros_interface',
            output='screen',
            parameters=[config_file]   # 自动把 yaml 内容加载到节点参数空间
        ),
    ])