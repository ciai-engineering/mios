from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    robot_type = LaunchConfiguration("robot_type")
    arm_prefix = LaunchConfiguration("arm_prefix")
    namespace = LaunchConfiguration("namespace")
    load_gripper = LaunchConfiguration("load_gripper")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    controller = LaunchConfiguration("controller")
    start_cartesian_controller = LaunchConfiguration("start_cartesian_controller")
    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("mios_franka_ros2"), "config", "controllers.yaml"]
    )

    franka = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("franka_bringup"), "launch", "franka.launch.py"]
            )
        ),
        launch_arguments={
            "robot_ip": robot_ip,
            "robot_type": robot_type,
            "arm_prefix": arm_prefix,
            "namespace": namespace,
            "load_gripper": load_gripper,
            "use_fake_hardware": use_fake_hardware,
            "fake_sensor_commands": "false",
            "joint_state_rate": "100",
            "controllers_yaml": controllers_yaml,
        }.items(),
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[controller, "--controller-manager-timeout", "30"],
        output="screen",
        condition=IfCondition(start_cartesian_controller),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip", description="FCI address of the robot"),
            DeclareLaunchArgument("robot_type", default_value="fr3"),
            DeclareLaunchArgument("arm_prefix", default_value=""),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("load_gripper", default_value="true"),
            DeclareLaunchArgument("use_fake_hardware", default_value="false"),
            DeclareLaunchArgument(
                "controller", default_value="mios_cartesian_pose_controller"
            ),
            DeclareLaunchArgument("start_cartesian_controller", default_value="true"),
            franka,
            controller_spawner,
        ]
    )
