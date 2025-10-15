import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Paquetes
    urdf_launch_pkg = FindPackageShare('urdf_launch')
    gpg_urdf_pkg = FindPackageShare('gpg_urdf')

    # Argumentos de launch
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=PathJoinSubstitution([gpg_urdf_pkg, 'urdf', 'gpg.urdf.xacro']),
        description='Path to robot URDF/XACRO file (relative to gpg_urdf package)'
    )
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',
        choices=['true', 'false'],
        description='Enable joint_state_publisher_gui'
    )

    # Añadir argumentos
    ld.add_action(model_arg)
    ld.add_action(gui_arg)

    # Incluir launch display.launch.py de urdf_launch
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([urdf_launch_pkg, 'launch', 'display.launch.py'])
        ]),
        launch_arguments={
            'urdf_package': 'gpg_urdf',
            'urdf_package_path': LaunchConfiguration('model'),
            'rviz_config': '',  # Ruta a RViz config opcional
            'jsp_gui': LaunchConfiguration('gui')
        }.items()
    )
    ld.add_action(display_launch)

    # ------------------------------

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items()
    )
    ld.add_action(gazebo)

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            "-topic", "/robot_description",
            "-entity", "robot",
        ],
        output='screen'
    )
    ld.add_action(spawn_robot)

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
        output='screen'
    )

    #ld.add_action(diff_drive_spawner)

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output='screen'
    )

    #ld.add_action(joint_state_broadcaster_spawner)

    return ld
