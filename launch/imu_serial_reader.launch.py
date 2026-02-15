from launch import LaunchDescription
from launch_ros.actions import Node
import os
import yaml

def generate_launch_description():
    # 소스 디렉토리에서 filter_config 로드 (빌드 없이 바로 반영)
    pkg_src = os.path.realpath(
        os.path.join(os.path.dirname(os.path.realpath(__file__)), os.pardir)
    )

    param_file_path = os.path.join(pkg_src, 'config', 'param.yaml')
    filter_config_path = os.path.join(pkg_src, 'config', 'filter_config')

    param_files = [param_file_path]
    if os.path.exists(filter_config_path):
        with open(filter_config_path, 'r') as f:
            tuned_params = yaml.safe_load(f)
        if tuned_params:
            tuned_params = {k: v for k, v in tuned_params.items()
                           if not k.startswith('qos_overrides') and k != 'use_sim_time'}
            param_files.append(tuned_params)

    imu_serial_reader_node = Node(
        package='imu_serial_reader',
        executable='imu_serial_receiver_node',
        name='imu_serial_receiver',
        output='screen',
        parameters=param_files
    )

    return LaunchDescription([
        imu_serial_reader_node
    ])
