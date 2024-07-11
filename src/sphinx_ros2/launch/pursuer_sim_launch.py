from launch import LaunchDescription
from launch_ros.actions import Node


my_package = "sphinx_ros2"
def generate_launch_description():
    pursuer = Node(
        package= my_package,
        executable='af_pursuer_sim',
        name='pursuer_sim',
    )
    
    control = Node(
        package= my_package,
        executable='af_control_sim',
        name='control_sim',
    )
    
    
    return LaunchDescription([
        pursuer,
        control
    ])
