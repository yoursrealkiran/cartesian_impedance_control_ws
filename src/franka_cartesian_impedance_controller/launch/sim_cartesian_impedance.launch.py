import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def robot_state_publisher_spawner(context: LaunchContext, arm_id, load_gripper, ee_id):
    arm_id_str = context.perform_substitution(arm_id)
    load_gripper_str = context.perform_substitution(load_gripper)
    ee_id_str = context.perform_substitution(ee_id)

    # Load xacro file and generate robot_description
    franka_xacro_filepath = os.path.join(
        get_package_share_directory("franka_description"),
        "robots",
        arm_id_str,
        arm_id_str + ".urdf.xacro",
    )

    robot_description = xacro.process_file(
        franka_xacro_filepath,
        mappings={
            "hand": load_gripper_str,
            "ee_id": ee_id_str,
            "ros2_control": "true",
            "use_fake_hardware": "true"
        }
    ).toprettyxml(indent="  ")

    # Load the controller config YAML
    controller_config = os.path.join(
        get_package_share_directory("franka_cartesian_impedance_controller"),
        "config",
        "cartesian_impedance_controllers.yaml"
    )

    return [
        # robot_state_publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),

        # controller manager node
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[{"robot_description": robot_description}, controller_config],
            output="screen"
        ),

        # spawn controllers after delay
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=["ros2", "run", "controller_manager", "spawner", "cartesian_impedance_controller"],
                    output="screen"
                ),
                ExecuteProcess(
                    cmd=["ros2", "run", "controller_manager", "spawner", "joint_state_broadcaster"],
                    output="screen"
                )
            ]
        ),

        # spawn robot into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', arm_id_str, '-topic', 'robot_description'],
            output='screen'
        )
    ]


def generate_launch_description():
    load_gripper_param = LaunchConfiguration("load_gripper")
    ee_id_param = LaunchConfiguration("ee_id")
    arm_id_param = LaunchConfiguration("arm_id")

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py"
            )
        )
    )

    return LaunchDescription([
        # Launch args
        DeclareLaunchArgument(
            "load_gripper",
            default_value="true",
            description="Whether to include the gripper"
        ),
        DeclareLaunchArgument(
            "ee_id",
            default_value="franka_hand",
            description="End-effector type"
        ),
        DeclareLaunchArgument(
            "arm_id",
            description="Robot arm ID (e.g., fr3, fp3, fer)"
        ),

        # Launch Gazebo
        gazebo,

        # Launch robot + control system
        OpaqueFunction(
            function=robot_state_publisher_spawner,
            args=[arm_id_param, load_gripper_param, ee_id_param]
        ),

        # GUI + RViz
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
        ),
    ])
