from launch import LaunchContext, LaunchDescription
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    lc = LaunchContext()
   


    node_lift = Node(
    package="wetexplorer_hardware",
    executable="lifting_joint_interface",
    name="lifting_joint_interface",
    parameters=[
        {"device_name": "/dev/ttyLift"},
        {"baudrate": 115200}
    ]
    )


    ld = LaunchDescription()
    ld.add_action(node_lift)   
    return ld
