from launch import LaunchDescription
from launch_ros.actions import Node


my_package = "anafi_ros2"
def generate_launch_description():
    frame_pub = Node(
        package= my_package,
        executable='af_frame_pub',
        name='frame_pub',
    )
    
    tracker = Node(
        package= my_package,
        executable='af_tracker',
        name='tracker',
    )

    bbox = Node(
        package= my_package,
        executable='af_3D_bbox',
        name='bbox_3D',
    )

    pnp = Node(
        package= my_package,
        executable='af_pnp',
        name='pnp_pose',
    )
    
    
    return LaunchDescription([
        frame_pub,
        tracker,
        bbox,
        pnp
    ])
