import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!
    world_file_name = "gcamp_world.world"
    package_name='lime_fizzio'
    pkg_path = os.path.join(get_package_share_directory(package_name))
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')  
    world_path = os.path.join(pkg_path, "worlds", world_file_name)

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

     # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path}.items()
    )

    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'lime_fizzio'],
                        output='screen')
    
    scan_to_pointcloud = Node(
        package='scan_to_pointcloud',
        executable='scan_to_pointcloud',
        parameters=[],
        arguments=[],
        output="screen",
    )

    kiss_icp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('kiss_icp'), 'launch','odometry.launch.py')),
        launch_arguments={'topic':'/cloud'}.items()
    )

    cartographer = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','cartographer.launch.py'
                )])
    )
    

    # Launch them all!
    return LaunchDescription([
        rsp,
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        spawn_entity,
        scan_to_pointcloud,
        kiss_icp,
        # cartographer,
    ])