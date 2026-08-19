import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _read_map_dims(map_file):
    with open(map_file) as f:
        lines = f.read().splitlines()
    height = int(lines[1].split()[1])
    width = int(lines[2].split()[1])
    return width, height


def _launch_setup(context, *args, **kwargs):
    bringup_share = get_package_share_directory('concpp_bringup')

    map_file = LaunchConfiguration('map_file').perform(context)
    if not os.path.isabs(map_file):
        map_file = os.path.join(bringup_share, 'maps', map_file)
    width, height = _read_map_dims(map_file)

    tau = float(LaunchConfiguration('tau').perform(context))
    seed = int(LaunchConfiguration('seed').perform(context))
    comm_range = float(LaunchConfiguration('comm_range').perform(context))

    base_xs = [int(v) for v in LaunchConfiguration('base_xs').perform(context).split(',')]
    base_ys = [int(v) for v in LaunchConfiguration('base_ys').perform(context).split(',')]
    robots_per_base = [int(v) for v in
                        LaunchConfiguration('robots_per_base').perform(context).split(',')]

    if not (len(base_xs) == len(base_ys) == len(robots_per_base)):
        raise RuntimeError(
            f"base_xs ({len(base_xs)}), base_ys ({len(base_ys)}), and "
            f"robots_per_base ({len(robots_per_base)}) must all list the same "
            f"number of comma-separated values -- one entry per base station")

    num_robots = sum(robots_per_base)   # computed ONCE here, passed to both nodes below --
                                          # same reasoning as width/height: one source of
                                          # truth instead of two processes agreeing by hand
    rviz_config = os.path.join(bringup_share, 'rviz', 'concpp.rviz')

    nodes = [
        Node(
            package='concpp_world_sim',
            executable='world_sim_node',
            name='world_sim',
            output='screen',
            parameters=[{
                'map_file': map_file,
                'seed': seed,
                'comm_range': comm_range,
                'base_xs': base_xs,
                'base_ys': base_ys,
                'robots_per_base': robots_per_base,
            }],
        ),
        Node(
            package='concpp_planner',
            executable='cp_node',
            name='cp_node',
            output='screen',
            parameters=[{
                'num_robots': num_robots,
                'tau': tau,
                'width': width,
                'height': height,
                'comm_range': comm_range,
                'base_xs': base_xs,
                'base_ys': base_ys,
            }],
        ),
    ]

    for rid in range(num_robots):
        nodes.append(Node(
            package='concpp_robot',
            executable='robot_node',
            name=f'robot_{rid}',
            output='screen',
            parameters=[{'robot_id': rid}],
        ))

    nodes.append(TimerAction(
        period=2.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
        )],
    ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file', default_value='small_test.map',
            description='Map filename (looked up in concpp_bringup/maps/) or an absolute path'),
        DeclareLaunchArgument('tau', default_value='1.0'),
        DeclareLaunchArgument('seed', default_value='42'),
        DeclareLaunchArgument('comm_range', default_value='5.0'),
        DeclareLaunchArgument(
            'base_xs', default_value='0',
            description='Comma-separated base x-coordinates, e.g. "0,20,40"'),
        DeclareLaunchArgument(
            'base_ys', default_value='0',
            description='Comma-separated base y-coordinates, matching base_xs one-to-one'),
        DeclareLaunchArgument(
            'robots_per_base', default_value='4',
            description='Comma-separated robot count per base, matching base_xs one-to-one'),
        OpaqueFunction(function=_launch_setup),
    ])
