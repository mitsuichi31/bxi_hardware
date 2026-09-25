"""1軸の実機試験（ベンチ）: BxiSystemInterface + joint_state_broadcaster + joint1_command。

起動直後は指令が未入力（NaN）なので、ハードウェアは Kp=Kd=0 の指令だけを送り、モーターは力を出さない。
位置を動かすときは ramp_command.py を使う（目標位置を少しずつ動かす）。

  ros2 launch bxi_hardware single_joint_hil.launch.py
  ros2 launch bxi_hardware single_joint_hil.launch.py config_file:=/path/to/bxi_hardware.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("bxi_hardware")
    hil_dir = PathJoinSubstitution([share, "hil", "single_joint"])
    config_file = LaunchConfiguration("config_file")
    spec_dir = LaunchConfiguration("spec_dir")

    robot_description = ParameterValue(
        Command([
            "xacro ", PathJoinSubstitution([hil_dir, "single_joint.urdf.xacro"]),
            " config_file:=", config_file,
            " spec_dir:=", spec_dir,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([hil_dir, "bxi_hardware.yaml"]),
            description="BXI hardware config（CAN bus / can_id / limit）"),
        DeclareLaunchArgument(
            "spec_dir",
            default_value=PathJoinSubstitution([share, "config", "specs"]),
            description="モーター機種 spec（MOTOR_70.yaml など）のディレクトリ"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[PathJoinSubstitution([hil_dir, "controllers.yaml"])],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint1_command"],
        ),
    ])
