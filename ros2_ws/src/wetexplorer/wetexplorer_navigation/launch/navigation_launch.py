# src/ring_navigation_launch.py

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml
def generate_launch_description():
  
    pkg_dir = get_package_share_directory("wetexplorer_navigation")
    # If AMENT_PREFIX_PATH is set (by sourcing), this locates the package.
    ekf_local_params   = os.path.join(pkg_dir, "config", "odom_ekf.yaml")
    ekf_global_params  = os.path.join(pkg_dir, "config", "global_ekf.yaml")
    navsat_params      = os.path.join(pkg_dir, "config", "navsat.yaml")

    # Read environment variables (defaults: sim="true", ref="map")
    sim_env = os.getenv("sim", "true").lower() == "true"
    
    ref_env = os.getenv("ref", "map").lower()        # either "map" or "odom"    
    if ref_env not in ["odom", "map"]:
        ref_env = "map"


    ld = LaunchDescription()

    
    # 1) Local EKF (node_6d_pose)
    ekf_local =  Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_node_odom",
            output="screen",
            parameters=[ekf_local_params, {"use_sim_time": sim_env}],
            remappings=[("odometry/filtered", "odometry/local")],
            
        )
    ld.add_action(ekf_local)

    # 2) If ref == "map": run global EKF, navsat_transform, gps_noise
    if ref_env == "map":
        ekf_global = Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_global_node",
            output="screen",
            parameters=[ekf_global_params, {"use_sim_time": sim_env}],
            remappings=[("odometry/filtered", "odometry/global")],
        )
        ld.add_action(ekf_global)

        navsat_transform =  Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[navsat_params, {"use_sim_time": sim_env}],
                remappings=[
                    ("imu", "imu_heading/data"),
                    ("gps/fix", "navsat/fix_cov"),
                    ("gps/filtered", "gps/filtered"),
                    ("odometry/gps", "odometry/gps"),
                    ("odometry/filtered", "odometry/global"),
                ],)
        ld.add_action(navsat_transform)

        gps_noise = Node(
            package="wetexplorer_navigation",
            executable="gps_noise_node",
            name="gps_noise_node",
            output="screen",
            parameters=[{"use_sim_time": sim_env}],
           
            
        )
        ld.add_action(gps_noise)

    # 3) Forward kinematics: sim vs real
    if sim_env:
        forward_kinematics_sim = Node(
            package="wetexplorer_control",
            executable="forward_kinematics_sim",
            name="forward_kinematics",
            output="screen",
            parameters=[{"use_sim_time": sim_env}],           
            
        )
        ld.add_action(forward_kinematics_sim)
    else:
        forward_kinematics = Node(
            package="wetexplorer_control",
            executable="forward_kinematics_node",
            name="forward_kinematics",
            output="screen",
            parameters=[{"use_sim_time": sim_env}],
        )
        ld.add_action(forward_kinematics)

    # 4) If sim is true: launch simulation-only nodes
    if sim_env:
        navsat_truth = Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform_truth",
                output="screen",
                parameters=[navsat_params, {"use_sim_time": sim_env}],
                remappings=[
                    ("imu", "imu_heading/data"),
                    ("gps/fix", "gps/ref"),
                    ("gps/filtered", "gps/filtered_2"),
                    ("odometry/gps", "odometry/gps_ref"),
                    ("odometry/filtered", "odometry/ground_truth"),
                ],)
        #ld.add_action(navsat_truth)
        ground_truth = Node(
            package="wetexplorer_navigation",
            executable="ground_truth_node",
            name="ground_truth_node",
            output="screen",
            parameters=[{"use_sim_time": sim_env}],          
            
        )
        ld.add_action(ground_truth)

        error_display = Node(
            package="wetexplorer_navigation",
            executable="error_display_node",
            name="error_display_node",
            output="screen",
            parameters=[{"use_sim_time": sim_env}],
        )
        ld.add_action(error_display)

        imu_filtering = Node(
            package="wetexplorer_navigation",
            executable="imu_filtering_node",
            name="imu_filtering_node",
            output="screen",
            parameters=[{"use_sim_time": sim_env}],
        )
        ld.add_action(imu_filtering)

    # 5) Core navigation nodes (always), injecting `ref`.
    position_controller = Node(
        package="wetexplorer_navigation",
        executable="position_controller_node",
        name="position_controller",
        output="screen",
        parameters=[
            {"use_sim_time": sim_env},
            {"ref": ref_env}
        ]
    )
    ld.add_action(position_controller)

    spin_control_action_server = Node(
        package="wetexplorer_navigation",
        executable="spin_control_action_server",
        name="spin_control_action_server",
        output="screen",
        parameters=[
            {"use_sim_time": sim_env},
            {"ref": ref_env}
        ]
    )
    ld.add_action(spin_control_action_server)

    

    goal_publisher = Node(
        package="wetexplorer_navigation",
        executable="goal_publisher",
        name="goal_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": sim_env},
            {"ref": ref_env}
        ]
    )
    ld.add_action(goal_publisher)

    map_server = Node(
        package="wetexplorer_navigation",
        executable="map_server_node",
        name="map_server",
        output="screen",
        parameters=[
            {"use_sim_time": sim_env},
            {"ref": ref_env}
        ]
    )
    ld.add_action(map_server)

    imu_covariance = Node(
        package="wetexplorer_navigation",
        executable="imu_covariance_node",
        name="imu_covariance_node",
        output="screen",
        parameters=[{"use_sim_time": sim_env}],
    )
    ld.add_action(imu_covariance)

    # 6) Dummy map & ring planner (always), also use `ref`
    dummy_map = Node(
        package="wetexplorer_navigation",
        executable="dummy_map_node",
        name="dummy_map_node",
        output="screen",
        parameters=[
            {"use_sim_time": sim_env},
            {"ref": ref_env}
        ]
    )
    #ld.add_action(dummy_map)

    ring_planner = Node(
        package="wetexplorer_navigation",
        executable="ring_path_planner",
        name="ring_path_planner",
        output="screen",
        parameters=[
            {"use_sim_time": sim_env},
            {"ref": ref_env}
        ]
    )
    ld.add_action(ring_planner)
    

    # Locate config directory inside wetexplorer_navigation
    pkg_dir = get_package_share_directory("wetexplorer_navigation")
    params_dir  = os.path.join(pkg_dir, "config")

    # Select appropriate Nav2 params file
    if ref_env == "odom":
        nav2_params_file = os.path.join(params_dir, "nav2_params_odom.yaml")
    else:
        nav2_params_file = os.path.join(params_dir, "nav2_params_map.yaml")

    # RewrittenYaml (no rewrites here, but preserves type safety)
    configured_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key="",
        param_rewrites={},
        convert_types=True
    )

    # Include the standard Nav2 bringup launch
    bringup_dir = get_package_share_directory("nav2_bringup")
    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "True",
            "params_file": configured_params,
            "autostart": "True"
        }.items(),
    )
    ld.add_action(navigation2_cmd)

   

    return ld
