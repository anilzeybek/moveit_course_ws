from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

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

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

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
        arguments=["-d", rviz_config_path]
    )

    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher_gui)
    ld.add_action(rviz2)

    return ld
