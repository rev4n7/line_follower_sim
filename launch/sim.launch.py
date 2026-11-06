"""Start Gazebo with the race track, the ROS <-> Gazebo bridge, and the line follower.

    ros2 launch line_follower_sim sim.launch.py
    ros2 launch line_follower_sim sim.launch.py headless:=true   # no windows (for testing)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('line_follower_sim')
    world = os.path.join(pkg, 'worlds', 'line_track.sdf')
    gui_config = os.path.join(pkg, 'config', 'gui.config')
    headless = LaunchConfiguration('headless')
    chase = LaunchConfiguration('chase_cam')

    gz_launch = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': f'-r {world} --gui-config {gui_config}',
                          'on_exit_shutdown': 'true'}.items(),
        condition=UnlessCondition(headless))
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': f'-r -s --headless-rendering {world}',
                          'on_exit_shutdown': 'true'}.items(),
        condition=IfCondition(headless))

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                   '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                   '/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                   '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])

    follower = Node(
        package='line_follower_sim', executable='follower', output='screen',
        parameters=[{'use_sim_time': True,
                     'show_window': PythonExpression(["'", headless, "' != 'true'"])}])

    # chase camera: ask the Gazebo GUI to follow the robot from behind and above
    follow = TimerAction(period=6.0, condition=IfCondition(PythonExpression(
        ["'", chase, "' == 'true' and '", headless, "' != 'true'"])), actions=[
        ExecuteProcess(cmd=['gz', 'service', '-s', '/gui/follow/offset', '--reqtype', 'gz.msgs.Vector3d',
                            '--reptype', 'gz.msgs.Boolean', '--timeout', '3000',
                            '--req', 'x: -1.6, y: 0.0, z: 1.1']),
        ExecuteProcess(cmd=['gz', 'service', '-s', '/gui/follow', '--reqtype', 'gz.msgs.StringMsg',
                            '--reptype', 'gz.msgs.Boolean', '--timeout', '3000',
                            '--req', 'data: "line_bot"']),
    ])

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('chase_cam', default_value='false',
                              description='true = Gazebo camera follows the robot'),
        gazebo_gui, gazebo_headless, bridge, follower, follow,
    ])
