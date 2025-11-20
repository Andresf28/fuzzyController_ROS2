import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    robotName = 'GoPiGo3'
    packageName = 'gpg_urdf'
    modelFilePath = 'urdf/gpg.urdf.xacro'

    # URDF
    pathModelFile = os.path.join(
        get_package_share_directory(packageName),
        modelFilePath
    )

    robotDescription = xacro.process_file(pathModelFile).toxml()

    # Gazebo sim
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py'
        )
    )

    WorldFileRelativePath = 'worlds/empty.world'
    pathWorldFile = os.path.join(
        get_package_share_directory(packageName),
        WorldFileRelativePath
    )

    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPackageLaunch,
        launch_arguments={
            'gz_args': ['-r ', pathWorldFile],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Spawn robot
    spawnModelNodeGazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
        '-topic', 'robot_description',
        '-name', robotName,
        '-x', '0.0',           
        '-y', '0.0',            
        '-z', '0.0',            
        '-R', '0.0',            
        '-P', '0.0',            
        '-Y', '-1.5708',       
    ],
        output='screen',
    )

    # Robot State Publisher
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robotDescription,
                'use_sim_time': True
            }
        ]
    )

    # Bridge config
    bridge_params = os.path.join(
        get_package_share_directory(packageName),
        'config',
        'bridge_parameters.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    # Controllers
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output='screen'
    )

    ground_truth_odom_node = Node(
        package='gpg_urdf',                
        executable='gz_odom',      
        name='ground_truth_odom',
        output='screen',
        parameters=[{'use_sim_time': True}]   
    )
    # -----------------------------------

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(gazeboLaunch)
    launchDescriptionObject.add_action(spawnModelNodeGazebo)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    launchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)
    launchDescriptionObject.add_action(diff_drive_spawner)
    launchDescriptionObject.add_action(joint_state_broadcaster_spawner)

    # Se agrega el nodo
    launchDescriptionObject.add_action(ground_truth_odom_node)

    return launchDescriptionObject
