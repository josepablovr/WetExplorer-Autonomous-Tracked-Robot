from launch import LaunchContext, LaunchDescription
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    lc = LaunchContext()
    joy_type = EnvironmentVariable('CPR_JOY_TYPE', default_value='ps4')

    sim_time = LaunchConfiguration('sim_time', default='true')
    declare_sim_time = DeclareLaunchArgument(
        'sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    filepath_config_joy = PathJoinSubstitution(
        [FindPackageShare('wetexplorer_control'), 'config', ('teleop_' + joy_type.perform(lc) + '.yaml')]
    )
    filepath_config_twist_mux = PathJoinSubstitution(
        [FindPackageShare('wetexplorer_control'), 'config', 'twist_mux.yaml']
    )

    common_params = [{'use_sim_time': sim_time}]

    node_joy = Node(
        namespace='joy_teleop',
        package='joy',
        executable='joy_node',
        output='screen',
        name='joy_node',
        parameters=[filepath_config_joy, *common_params]
    )

    node_teleop_twist_joy = Node(
        namespace='joy_teleop',
        package='teleop_twist_joy',
        executable='teleop_node',
        output='screen',
        name='teleop_twist_joy_node',
        parameters=[filepath_config_joy, *common_params]
    )

    node_inverse_kinematics = Node(        
        package='wetexplorer_control',
        executable='robot_control_node',
        output='screen',
        name='inverse_kinematics_node',
        parameters=common_params
    )

    node_safe_commands = Node(        
        package='wetexplorer_control',
        executable='safe_commands_node',
        output='screen',
        name='safe_commands_node',
        parameters=common_params
    )

    node_twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        parameters=[filepath_config_twist_mux, *common_params]
    )

    node_6d_pose = Node(
        package='wetexplorer_control',
        executable='6d_pose_caller',
        output='screen',
        name='object_pose_caller',
        parameters=common_params
    )

    node_joy_lift = Node(
        package='wetexplorer_control',
        executable='joy_lift_controller',
        output='screen',
        name='joy_lift',
        parameters=common_params
    )

     
    
    node_depth_mux = Node(
        package='topic_tools',
        executable='mux',
        name='mux_depth',
        output='screen',
        arguments=[
            '/camera/depth/raw',
            '/camera1/depth_raw',
            '/camera2/depth_raw'
        ],
        parameters=common_params
    )

    node_image_mux = Node(
        package='topic_tools',
        executable='mux',
        name='mux_color',
        output='screen',
        arguments=[
            '/camera/image/raw',
            '/camera1/image_raw',
            '/camera2/image_raw'
        ],
        parameters=common_params
    )

    node_camera_info_mux = Node(
        package='topic_tools',
        executable='mux',
        name='mux_info',
        output='screen',
        arguments=[
            '/camera/camera_info',
            '/camera1/camera_info',
            '/camera2/camera_info'
        ],
        parameters=common_params
    )

    ld = LaunchDescription()
    ld.add_action(declare_sim_time)
    ld.add_action(node_joy)
    ld.add_action(node_teleop_twist_joy)
    ld.add_action(node_twist_mux)
    ld.add_action(node_inverse_kinematics)
    ld.add_action(node_safe_commands)
    ld.add_action(node_6d_pose)
    ld.add_action(node_joy_lift)
    
    ld.add_action(node_depth_mux)
    ld.add_action(node_image_mux)
    ld.add_action(node_camera_info_mux)
    
    return ld
