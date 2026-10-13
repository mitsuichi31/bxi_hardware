from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("bxi_hardware")
    hil_dir = str(Path(__file__).resolve().parent)
    config_file = LaunchConfiguration("config_file")
    spec_dir = LaunchConfiguration("spec_dir")
    controllers_file = LaunchConfiguration("controllers_file")

    robot_description = ParameterValue(
        Command([
            "xacro ", PathJoinSubstitution([hil_dir, "two_joint.urdf.xacro"]),
            " config_file:=", config_file,
            " spec_dir:=", spec_dir,
            " hardware_plugin:=", LaunchConfiguration("hardware_plugin"),
            " joint_lower:=", LaunchConfiguration("joint_lower"),
            " joint_upper:=", LaunchConfiguration("joint_upper"),
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument("joint_lower", default_value="-1.65"),
        DeclareLaunchArgument("joint_upper", default_value="1.65"),
        DeclareLaunchArgument("hardware_plugin", default_value="bxi_hardware/BxiSystemInterface",
                              description="HWIF plugin; synthetic trace benchmark requires vcan and test opt-in"),
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([hil_dir, "bxi_hardware_stream_1khz_2joint.yaml"]),
            description="BXI hardware config（CAN bus / can_id / limit）"),
        DeclareLaunchArgument(
            "spec_dir",
            default_value=PathJoinSubstitution([share, "config", "specs"]),
            description="モーター機種 spec（MOTOR_70.yaml など）のディレクトリ"),
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution([hil_dir, "controllers_1khz_2joint.yaml"]),
            description="controller_manager の設定（更新周期・RT優先度・controller）"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[controllers_file],
            output="screen",
            # Resolve every symbol at load time. With lazy binding, the first call of each function
            # (e.g. the first trajectory cycle) runs the dynamic linker inside update(): 35-92 µs
            # on vcan vs 4-10 µs with this set (log/rt_first_cycle_20260930).
            additional_env={"LD_BIND_NOW": "1"},
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
        ),
        Node(package="controller_manager", executable="spawner",
             arguments=["joint1_command", "joint2_command", "--activate-as-group"]),
    ])
