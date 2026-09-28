from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    static_marker_0 = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_marker_0',
            arguments=['0.0', '0.0', '0.1', '0.0', '0.0', '0', 'map', 'marker_frame_0']
        )

    static_marker_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_marker_1',
        arguments=['2.0', '0.0', '0.1', '1.5708', '0.0', '0.0', 'map', 'marker_frame_1']
    )

    static_marker_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_marker_2',
        arguments=['2.0', '2.0', '0.1', '3.14159', '0.0', '0.0', 'map', 'marker_frame_2']
    )

    static_marker_3 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_marker_3',
        arguments=['0.0', '2.0', '0.1', '-1.5708', '0.0', '0.0', 'map', 'marker_frame_3']
    )
        
    return LaunchDescription([
        static_marker_0,
        static_marker_1,
        static_marker_2,
        static_marker_3,
    ])
