from launch import LaunchContext, LaunchDescription
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable, 
                            IncludeLaunchDescription, SetLaunchConfiguration)
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
def generate_launch_description():
    """Launch WetExplorer control stack with optional simulated clock.

    The choice between simulated and wall time is controlled via the **environment
    variable** ``sim`` (default: ``"true"``).  Any non‑truthy value ("false", "0",
    etc.) disables ``use_sim_time`` for all nodes.
    """

    # A LaunchContext lets us resolve substitutions that depend on env‑vars early.
    lc = LaunchContext()

    # Which joystick mapping to use (defaults to ps4 if CPR_JOY_TYPE is undefined).
    joy_type = EnvironmentVariable("CPR_JOY_TYPE", default_value="ps4")

    # ──────────────────────────────────────────────────────────────────────────────
    # Sim‑time flag (driven by env‑var "sim")
    # ──────────────────────────────────────────────────────────────────────────────
    sim_env = os.getenv("sim", "true").lower() == "true"
    sim_time_param = {"use_sim_time": sim_env}

    # ──────────────────────────────────────────────────────────────────────────────
    # Parameter & config file paths
    # ──────────────────────────────────────────────────────────────────────────────
    filepath_config_joy = PathJoinSubstitution([
        FindPackageShare("wetexplorer_control"),
        "config",
        f"teleop_{joy_type.perform(lc)}.yaml",
    ])

    filepath_config_twist_mux = PathJoinSubstitution([
        FindPackageShare("wetexplorer_control"),
        "config",
        "twist_mux.yaml",
    ])
    ld = LaunchDescription()

    # ──────────────────────────────────────────────────────────────────────────────
    # Nodes
    # ──────────────────────────────────────────────────────────────────────────────
    node_joy = Node(
        namespace="joy_teleop",
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[filepath_config_joy, sim_time_param],
    )

    node_teleop_twist_joy = Node(
        namespace="joy_teleop",
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        output="screen",
        parameters=[filepath_config_joy, sim_time_param],
    )

    node_6d_pose = Node(
        package="wetexplorer_navigation",
        executable="6d_pose_caller",
        name="object_pose_caller",
        output="screen",
        parameters=[sim_time_param],
    )
    ld.add_action(node_6d_pose)

    node_inverse_kinematics = Node(
        package="wetexplorer_control",
        executable="robot_control_node",
        name="inverse_kinematics_node",
        output="screen",
        parameters=[sim_time_param],
    )

    node_safe_commands = Node(
        package="wetexplorer_control",
        executable="safe_commands_node",
        name="safe_commands_node",
        output="screen",
        parameters=[sim_time_param],
    )

    node_twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[filepath_config_twist_mux, sim_time_param],
    )

    
    node_joy_lift = Node(
        package="wetexplorer_control",
        executable="joy_lift_controller",
        name="joy_lift",
        output="screen",
        parameters=[sim_time_param],
    )

    if not sim_env:
        print("USING REAL TIME PARAMETERS")
        robot_description = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(
                        get_package_share_directory('wetexplorer_description' ),'launch','description_tracks.launch.py'
                    )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'false'}.items()
        )
        ld.add_action(robot_description)
    else:
        print("USING SIM TIME PARAMETERS")

        # Node for transforming IMU frames
        imu_transformer = Node(
            package='imu_transformer',
            executable='imu_transformer_node',
            name='imu_data_transformer',
            output='screen',
            remappings=[
                ('imu_in', 'imu/data_raw'),
                ('imu_out', 'imu/data_transformed')
            ],
            parameters=[{
                'target_frame': 'imu_link_target'  # check this name
            }]
        )

        # Node for signal processing
        imu_signal_processing = Node(
            package='wetexplorer_navigation',
            executable='imu_filtering_node',
            name='imu_filtering_node',
            output='screen'
        )

        ld.add_action(imu_transformer)
        ld.add_action(imu_signal_processing)


        

    # ──────────────────────────────────────────────────────────────────────────────
    # Launch description
    # ──────────────────────────────────────────────────────────────────────────────
    
    # Tele‑op first for readability
    ld.add_action(node_joy)
    ld.add_action(node_teleop_twist_joy)
    ld.add_action(node_twist_mux)

    ld.add_action(node_inverse_kinematics)
    ld.add_action(node_safe_commands)
    ld.add_action(node_joy_lift)


    return ld
