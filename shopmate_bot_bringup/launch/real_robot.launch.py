import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Package Directories
    pkg_shopmate_description = get_package_share_directory('shopmate_bot_description')
    pkg_rplidar = get_package_share_directory('rplidar_ros')

    # 1. Process the URDF file
    xacro_file = os.path.join(pkg_shopmate_description, 'urdf', 'shopmate_bot.xacro')
    robot_description = {'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str)}

    # RViz Configuration Path 
    # (Assuming you saved lidar_scan.rviz inside the description package's config folder)
    rviz_config_path = os.path.join(pkg_shopmate_description, 'config', 'lidar_scan.rviz')

    # 2. Robot State Publisher (Loads the TF tree for the real robot)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': False}] 
    )

    # 3. Joint State Publisher (Fills in blank joint data so the robot model renders!)
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}]
    )

    # 4. RPLidar C1 Node
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rplidar, 'launch', 'rplidar_c1_launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0', 
            'frame_id': 'lidar_link_1'
        }.items()
    )

    # 5. RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': False}] 
    )

    return LaunchDescription([
        rsp_node,
        jsp_node,
        lidar_launch,
        rviz_node
    ])