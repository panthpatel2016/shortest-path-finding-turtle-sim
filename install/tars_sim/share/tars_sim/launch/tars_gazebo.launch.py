from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('tars_sim')
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'tars.urdf.xacro'])
    controllers_yaml = PathJoinSubstitution([pkg_share, 'config', 'controllers.yaml'])

    # Run robot_state_publisher with xacro-expanded URDF
    robot_description = Command(['xacro ', urdf_file])

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Start Gazebo Classic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'), '/launch/gazebo.launch.py']
        ),
        launch_arguments={'verbose': 'true'}.items()
    )

    # Spawn the robot into Gazebo from /robot_description
    spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'tars'],
        output='screen'
    )

    # Start controllers
    jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']
    )
    traj = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['tars_controller', '--controller-manager', '/controller_manager',
                   '--param-file', controllers_yaml]
    )

    return LaunchDescription([gazebo, rsp, spawner, jsb, traj])
