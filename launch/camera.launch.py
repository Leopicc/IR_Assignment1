from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    
    container = ComposableNodeContainer(
        name='apriltag_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            
            # APRILTAG NODE
            ComposableNode(
                package='apriltag_ros',
                plugin='AprilTagNode',
                name='apriltag',
                namespace='apriltag',
                remappings=[
                    ("image_rect", "/rgb_camera/image"),
                    ("camera_info", "/rgb_camera/camera_info")
                ],
                parameters=[
                    {"family": "36h11"},
                    {"size": 0.050},
                    {"max_hamming": 0},
                    {"use_sim_time": True}
                ]
            ),
        ],
        parameters=[{"use_sim_time": True}],
        output='screen'
    )

    return LaunchDescription([container])
##########

