from launch import LaunchContext, LaunchDescription
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    lc = LaunchContext()
    joy_type = EnvironmentVariable('CPR_JOY_TYPE', default_value='ps4')


    filepath_config_joy = PathJoinSubstitution(
        [FindPackageShare('wetexplorer_control'), 'config', ('teleop_' + joy_type.perform(lc) + '.yaml')]
    )
    filepath_config_twist_mux = PathJoinSubstitution(
        [FindPackageShare('wetexplorer_control'), 'config', 'twist_mux.yaml']
    )

    node_joy = Node(
        namespace='joy_teleop',
        package='joy',
        executable='joy_node',
        output='screen',
        name='joy_node',
        parameters=[filepath_config_joy]
    )

    node_teleop_twist_joy = Node(
        namespace='joy_teleop',
        package='teleop_twist_joy',
        executable='teleop_node',
        output='screen',
        name='teleop_twist_joy_node',
        parameters=[filepath_config_joy]
    )


    node_inverse_kinematics = Node(        
        package='wetexplorer_control',
        executable='robot_control_node',
        output='screen',
        name='inverse_kinematics_node')

    node_safe_commands = Node(        
        package='wetexplorer_control',
        executable='safe_commands_node',
        output='screen',
        name='safe_commands_node')

    node_twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        parameters=[filepath_config_twist_mux]
    )

    node_6d_pose = Node(
        package='wetexplorer_control',
        executable='6d_pose_caller',
        output='screen',
        name='object_pose_caller'
    )

    node_joy_lift = Node(
        package='wetexplorer_control',
        executable='joy_lift_controller',
        output='screen',
        name='joy_lift'
    )


    ld = LaunchDescription()
    ld.add_action(node_joy)
    ld.add_action(node_teleop_twist_joy)
    ld.add_action(node_twist_mux)
    ld.add_action(node_inverse_kinematics)
    ld.add_action(node_safe_commands)
    #ld.add_action(node_6d_pose)
    ld.add_action(node_joy_lift)
    return ld
