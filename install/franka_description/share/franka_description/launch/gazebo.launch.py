import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def gazebo_robot_spawner(context: LaunchContext, arm_id, load_gripper, ee_id):
    arm_id_str = context.perform_substitution(arm_id)
    load_gripper_str = context.perform_substitution(load_gripper)
    ee_id_str = context.perform_substitution(ee_id)

    xacro_file = os.path.join(
        get_package_share_directory("franka_description"),
        "robots",
        arm_id_str,
        arm_id_str + ".urdf.xacro",
    )

    robot_description = xacro.process_file(
        xacro_file, mappings={"hand": load_gripper_str, "ee_id": ee_id_str}
    ).toxml()

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-topic", "robot_description", "-entity", arm_id_str],
            output="screen",
        ),
    ]


def generate_launch_description():
    # Declare launch arguments
    load_gripper_arg = DeclareLaunchArgument(
        "load_gripper", default_value="true", description="Load the Franka hand"
    )
    ee_id_arg = DeclareLaunchArgument(
        "ee_id", default_value="franka_hand", description="End-effector type"
    )
    arm_id_arg = DeclareLaunchArgument(
        "arm_id", default_value="fr3", description="Franka arm ID"
    )

    arm_id = LaunchConfiguration("arm_id")
    load_gripper = LaunchConfiguration("load_gripper")
    ee_id = LaunchConfiguration("ee_id")

    gazebo_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        )
    )

    robot_spawner = OpaqueFunction(
        function=gazebo_robot_spawner, args=[arm_id, load_gripper, ee_id]
    )

    return LaunchDescription([
        load_gripper_arg,
        ee_id_arg,
        arm_id_arg,
        gazebo_launcher,
        robot_spawner,
    ])
