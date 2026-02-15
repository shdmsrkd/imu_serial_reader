from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # 소스 디렉토리 기준 경로 (빌드 없이 바로 반영)
    pkg_src = os.path.realpath(
        os.path.join(os.path.dirname(os.path.realpath(__file__)), os.pardir)
    )

    param_file_path = os.path.join(pkg_src, 'config', 'param.yaml')
    rviz_config = os.path.join(pkg_src, 'rviz', 'default.rviz')

    imu_serial_reader_node = Node(
        package='imu_serial_reader',
        executable='imu_serial_receiver_node',
        name='imu_serial_receiver',
        output='screen',
        parameters=[param_file_path]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else []
    )

    rqt_reconfigure_node = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_reconfigure', 'rqt_reconfigure'],
        output='screen'
    )

    return LaunchDescription([
        imu_serial_reader_node,
        rviz2_node,
        rqt_reconfigure_node,
    ])
