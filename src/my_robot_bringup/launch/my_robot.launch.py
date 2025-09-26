from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    # Following are `robot_state_publisher` related:
    xacro_path = PathJoinSubstitution(
        [
            FindPackageShare("my_robot_description"),
            "urdf",
            "my_robot.urdf.xacro",
        ]
    )
    robot_description = Command(["xacro ", xacro_path])
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    # Following are ros2 controllers related:
    ros2_controllers_yaml_path = PathJoinSubstitution(
        [
            FindPackageShare("my_robot_bringup"),
            "config",
            "ros2_controllers.yaml",
        ]
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, ros2_controllers_yaml_path],
    )
    spawner1 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    spawner2 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
    )
    spawner3 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
    ) 

    # Following just spawns the moveit:
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("my_robot_moveit_config"),
                        "launch",
                        "move_group.launch.py",
                    ]
                )
            ]
        )
    )

    # Following are rviz related:
    rviz_config_path = PathJoinSubstitution(
        [
            FindPackageShare("my_robot_description"),
            "rviz",
            "urdf_config.rviz",
        ]
    )
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_path],
    )

    ld.add_action(robot_state_publisher)
    ld.add_action(control_node)
    ld.add_action(spawner1)
    ld.add_action(spawner2)
    ld.add_action(spawner3)
    ld.add_action(moveit_launch)
    ld.add_action(rviz2)

    return ld
