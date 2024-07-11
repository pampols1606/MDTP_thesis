from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

my_package = "anafi_ros2"

def generate_launch_description():

    #Arguments
    display_2D = DeclareLaunchArgument(
        'display_2D',
        default_value='false',
        description='Detection display with 2D box'
    )

    display_3D = DeclareLaunchArgument(
        'display_3D',
        default_value='true',
        description='Detection display with 3D box'
    )

    filter_kp = DeclareLaunchArgument(
        'filter_kp',
        default_value='true',
        description='Keypoint filtering'
    )

    filter_pnp = DeclareLaunchArgument(
        'filter_pnp',
        default_value='true',
        description="Detected drone's position filtering"
    )

    pid_popup = DeclareLaunchArgument(
        'pid_popup',
        default_value='false',
        description="PID tuning popup window"
    )

    ss_tracking = DeclareLaunchArgument(
        'ss_tracking',
        default_value='false',
        description="Steady-state pursuing"
    )

    #Nodes
    tracker = Node(
        package=my_package,
        executable='af_tracker',
        name='tracker',
        parameters=[{'display_2D': LaunchConfiguration('display_2D')}]
    )

    bbox = Node(
        package=my_package,
        executable='af_3D_bbox',
        name='bbox_3D',
        parameters=[
            {'display_3D': LaunchConfiguration('display_3D')},
            {'filter_kp': LaunchConfiguration('filter_kp')}
        ]
    )

    pnp = Node(
        package=my_package,
        executable='af_pnp',
        name='pnp_pose',
        parameters=[{'filter_pnp': LaunchConfiguration('filter_pnp')}]
    )

    control = Node(
        package=my_package,
        executable='af_control',
        name='control',
        parameters=[
            {'pid_popup': LaunchConfiguration('pid_popup')},
            {'ss_tracking': LaunchConfiguration('ss_tracking')}
        ]
    )

    return LaunchDescription([
        # Arguments
        display_2D,
        display_3D,
        filter_kp,
        filter_pnp,
        pid_popup,
        ss_tracking,

        # Nodes
        tracker,
        bbox,
        pnp,
        control
    ])
