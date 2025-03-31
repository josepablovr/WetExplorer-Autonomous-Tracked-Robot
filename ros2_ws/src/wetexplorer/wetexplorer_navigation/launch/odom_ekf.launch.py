# Copyright 2018 Open Source Robotics Foundation, Inc.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
import os
import launch.actions


def generate_launch_description():
    pkg_dir = get_package_share_directory(
        "wetexplorer_navigation")
    ekf_params = os.path.join(
        pkg_dir, "config", "odom_ekf.yaml")

    return LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                "output_final_position", default_value="false"
            ),
            
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_node_odom",
                output="screen",
                parameters=[ekf_params, {"use_sim_time": True}],
                remappings=[("odometry/filtered", "odometry/local")],
            ),
            launch_ros.actions.Node(
                package="wetexplorer_navigation",
                executable="imu_filtering_node",
                name="imu_noise_node",
                output="screen",
                parameters=[{"use_sim_time": True}],                
            ),
            launch_ros.actions.Node(
                package="wetexplorer_control",
                executable="forward_kinematics_sim",
                name="forward_kinematics",
                output="screen",
                parameters=[{"use_sim_time": True}],                
            ),
            
        ]
    )