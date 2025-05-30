from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory("wetexplorer_navigation")
    ekf_params = os.path.join(pkg_dir, "config", "odom_ekf.yaml")
    ekf_global_params = os.path.join(pkg_dir, "config", "global_ekf.yaml")
    navsat_params = os.path.join(pkg_dir, "config", "navsat.yaml")

    sim_time = LaunchConfiguration("sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "output_final_position", default_value="false"
        ),
        DeclareLaunchArgument(
            "sim_time",
            default_value="True",
            description="Use simulation time if true"
        ),

        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_node_odom",
            output="screen",
            parameters=[ekf_params, {"use_sim_time": sim_time}],
            remappings=[("odometry/filtered", "odometry/local")],
            
        ),


      

        Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[navsat_params, {"use_sim_time": True}],
                remappings=[
                    ("imu", "imu_heading/data"),
                    ("gps/fix", "navsat/fix_cov"),
                    ("gps/filtered", "gps/filtered"),
                    ("odometry/gps", "odometry/gps"),
                    ("odometry/filtered", "odometry/global"),
                ],),

        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_global_node",
            output="screen",
            parameters=[ekf_global_params, {"use_sim_time": sim_time}],
            remappings=[("odometry/filtered", "odometry/global")],
        ),

        # Run real robot version when sim_time == false
        Node(
            package="wetexplorer_control",
            executable="forward_kinematics_sim",
            name="forward_kinematics",
            output="screen",
            parameters=[{"use_sim_time": sim_time}],
           
            
        ),

        Node(
            package="wetexplorer_navigation",
            executable="gps_noise_node",
            name="gps_noise_node",
            output="screen",
            parameters=[{"use_sim_time": sim_time}],
           
            
        ),

        Node(
            package="wetexplorer_navigation",
            executable="gps_noise_node",
            name="gps_noise_node",
            output="screen",
            parameters=[{"use_sim_time": sim_time}],

           
            
        ),
        Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform_truth",
                output="screen",
                parameters=[navsat_params, {"use_sim_time": True}],
                remappings=[
                    ("imu", "imu_heading/data"),
                    ("gps/fix", "gps/ref"),
                    ("gps/filtered", "gps/filtered_2"),
                    ("odometry/gps", "odometry/gps_ref"),
                    ("odometry/filtered", "odometry/ground_truth"),
                ],),
        Node(
            package="wetexplorer_navigation",
            executable="ground_truth_node",
            name="ground_truth_node",
            output="screen",
            parameters=[{"use_sim_time": sim_time}],
           
            
        ),
        Node(
            package="wetexplorer_navigation",
            executable="error_display_node",
            name="error_display_node",
            output="screen",
            parameters=[{"use_sim_time": sim_time}],
           
            
        ),

        


        # Run sim version when sim_time == false
        
        Node(
            package="wetexplorer_navigation",
            executable="position_controller_node",
            name="position_control",
            output="screen",
            parameters=[{"use_sim_time": sim_time}],
        ),

        Node(
            package="wetexplorer_navigation",
            executable="goal_publisher",
            name="goal_pub",
            output="screen",
            parameters=[{"use_sim_time": sim_time}],
        ),
        Node(
            package="wetexplorer_navigation",
            executable="map_server_node",
            name="map_server",
            output="screen",
            parameters=[{"use_sim_time": sim_time}],
        ),
        Node(
        package='wetexplorer_navigation',
        executable='imu_covariance_node',
        name='imu_covariance_node',
        output='screen',
        parameters=[{"use_sim_time": sim_time}],
        ),

        Node(
        package='wetexplorer_navigation',
        executable='dummy_map_node',
        name='dummy_map_node',
        output='screen',
        parameters=[{"use_sim_time": sim_time}],
        ),

        Node(
        package='wetexplorer_navigation',
        executable='ring_path_planner',
        name='ring_path_planner',
        output='screen',
        parameters=[{"use_sim_time": sim_time}],
        ),

        Node(
        package='wetexplorer_navigation',
        executable='imu_filtering_node',
        name='imu_filtering_node',
        output='screen',
        parameters=[{"use_sim_time": sim_time}],
    )
    ])
