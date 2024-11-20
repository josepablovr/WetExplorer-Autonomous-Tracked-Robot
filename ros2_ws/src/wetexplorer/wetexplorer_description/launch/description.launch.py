import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import Node
from launch.substitutions import LaunchConfiguration
import xacro


def generate_launch_description():
    # Define the package and Xacro file to load
    namePackage = 'wetexplorer_description'
    xacroFileRelativePath = 'urdf/aluminium_frame.xacro'  # Relative path to your Xacro file
    
    # Absolute path to the Xacro file
    pathXacroFile = os.path.join(get_package_share_directory(namePackage), xacroFileRelativePath)
    
    # Get the robot description from the Xacro file
    robotDescription = xacro.process_file(pathXacroFile).toxml()

    # Robot State Publisher Node to publish the URDF
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription, 'use_sim_time': True}]
    )

    # Launch description object
    launchDescriptionObject = LaunchDescription()

    # Add the robot state publisher node
    launchDescriptionObject.add_action(nodeRobotStatePublisher)

    return launchDescriptionObject
