"""G-only launch; execution requires a separate G3/G4 approval."""
from pathlib import Path
import json
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def setup(context):
    directory = Path(LaunchConfiguration('config_dir').perform(context)).resolve()
    n = int(LaunchConfiguration('axes').perform(context))
    manifest = json.loads((directory / 'manifest.json').read_text())
    if n not in (2, 3) or manifest['axes'] != n:
        raise ValueError('axes does not match generated manifest')
    share = Path(get_package_share_directory('bxi_hardware'))
    description = xacro.process_file(str(share/'hil/n_axis/n_axis.urdf.xacro'), mappings={
        'axes': str(n), 'config_file': str(directory/'hardware.yaml'), 'spec_dir': str(share/'config/specs')}).toxml()
    worker = Path(manifest['output_dir'])/'hwif'
    worker.mkdir(exist_ok=False)
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': description}]),
        Node(package='controller_manager', executable='ros2_control_node', output='screen',
             parameters=[str(directory/'controllers.yaml')], remappings=[('~/robot_description','/robot_description')], additional_env={'LD_BIND_NOW': '1', 'BXI_WORKER_TIMING_DIR': str(worker)}),
        Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster']),
        Node(package='controller_manager', executable='spawner', arguments=['n_axis']),
    ]

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('axes', choices=['2', '3']),
        DeclareLaunchArgument('config_dir'), OpaqueFunction(function=setup)])
