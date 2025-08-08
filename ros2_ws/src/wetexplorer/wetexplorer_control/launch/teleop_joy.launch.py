from launch import LaunchContext, LaunchDescription
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


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
   

    if sim_env:
        camera1_topic_info = "/camera1/camera_info"
        camera2_topic_info = "/camera2/camera_info"
        camera1_topic_depth = "/camera1/depth_raw"
        camera2_topic_depth = "/camera2/depth_raw"
        camera1_topic_image = "/camera1/image_raw"
        camera2_topic_image = "/camera2/image_raw"
    else:
        camera1_topic_info = "/camera/depth/camera_info"
        camera2_topic_info = "/camera2/camera_info"
        camera1_topic_depth = "/camera/depth/image_raw"
        camera2_topic_depth = "/camera2/depth_raw"
        camera1_topic_image = "/camera/color/image_raw"
        camera2_topic_image = "/camera2/image_raw"
        


    node_depth_mux = Node(
        package="topic_tools",
        executable="mux",
        name="mux_depth",
        output="screen",
        arguments=[
            "/camera/depth/raw",
            camera1_topic_depth,
            camera2_topic_depth,
        ],
        parameters=[sim_time_param],
    )

    node_image_mux = Node(
        package="topic_tools",
        executable="mux",
        name="mux_color",
        output="screen",
        arguments=[
            "/camera/image/raw",
            camera1_topic_image,
            camera2_topic_image,
        ],
        parameters=[sim_time_param],
    )

    node_camera_info_mux = Node(
        package="topic_tools",
        executable="mux",
        name="mux_info",
        output="screen",
        arguments=[
            "/camera/camera_info",
            camera1_topic_info,
            camera2_topic_info,
        ],
        parameters=[sim_time_param],
    )

    # ──────────────────────────────────────────────────────────────────────────────
    # Launch description
    # ──────────────────────────────────────────────────────────────────────────────
    ld = LaunchDescription()

    # Tele‑op first for readability
    ld.add_action(node_joy)
    ld.add_action(node_teleop_twist_joy)
    ld.add_action(node_twist_mux)

    ld.add_action(node_inverse_kinematics)
    ld.add_action(node_safe_commands)
    ld.add_action(node_joy_lift)

    ld.add_action(node_depth_mux)
    ld.add_action(node_image_mux)
    ld.add_action(node_camera_info_mux)

    return ld
