from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    ir_launch_share = get_package_share_directory('ir_launch')
    ir_base_share   = get_package_share_directory('ir_base')
    launch_share    = get_package_share_directory('group11_assignament_1')

    # Simulazione
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ir_launch_share, 'launch', 'assignment_1.launch.py')
        ),
    )

    # Camera
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_share, 'launch', 'camera.launch.py')
        )
    )

    # Nodi
    common_params = [{'use_sim_time': True}]

    april_server = Node(
        package='group11_assignament_1',
        executable='april_detection_node',
        name='april_detection_node',
        output='screen',
        parameters=common_params,
    )

    table_server = Node(
        package='group11_assignament_1',
        executable='table_detection_node',
        name='table_detection_node',
        output='screen',
        parameters=common_params,
    )

    brain_node = Node(
        package='group11_assignament_1',
        executable='brain_node',
        name='brain_node',
        output='screen',
        parameters=common_params,
    )

    return LaunchDescription([
        simulation_launch,
        camera_launch,
        april_server,
        table_server,
        brain_node,
    ])
