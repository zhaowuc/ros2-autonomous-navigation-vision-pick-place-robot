from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    amcl_alpha = LaunchConfiguration('amcl_alpha')
    goal_xy_tolerance = LaunchConfiguration('goal_xy_tolerance')
    bt_xml = PathJoinSubstitution([
        FindPackageShare('roscar_nav'),
        'behavior_trees',
        'navigate_to_pose_safe.xml',
    ])
    configured = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={'use_sim_time': use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )
    common_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    lifecycle_nodes = [
        'map_server',
        'amcl',
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'velocity_smoother',
        'collision_monitor',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map'),
        DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('amcl_alpha', default_value='0.2'),
        DeclareLaunchArgument('goal_xy_tolerance', default_value='0.02'),
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            output='screen', parameters=[configured, {'yaml_filename': map_yaml}],
            remappings=common_remaps,
        ),
        Node(
            package='nav2_amcl', executable='amcl', name='amcl',
            output='screen', parameters=[configured, {
                'alpha1': ParameterValue(amcl_alpha, value_type=float),
                'alpha2': ParameterValue(amcl_alpha, value_type=float),
                'alpha3': ParameterValue(amcl_alpha, value_type=float),
                'alpha4': ParameterValue(amcl_alpha, value_type=float),
                'alpha5': ParameterValue(amcl_alpha, value_type=float),
            }], remappings=common_remaps,
        ),
        Node(
            package='nav2_controller', executable='controller_server',
            name='controller_server', output='screen', parameters=[configured, {
                'goal_checker.xy_goal_tolerance': ParameterValue(
                    goal_xy_tolerance, value_type=float
                ),
            }],
            arguments=['--ros-args', '--log-level', 'controller_server:=debug'],
            remappings=common_remaps + [('cmd_vel', '/cmd_vel_nav_controller')],
        ),
        Node(
            package='roscar_command_arbiter', executable='nav_command_limiter',
            name='roscar_nav_command_limiter', output='screen',
            parameters=[{'linear_limit': 0.20, 'angular_limit': 0.45}],
        ),
        Node(
            package='nav2_smoother', executable='smoother_server',
            name='smoother_server', output='screen', parameters=[configured],
            remappings=common_remaps,
        ),
        Node(
            package='nav2_planner', executable='planner_server',
            name='planner_server', output='screen', parameters=[configured],
            remappings=common_remaps,
        ),
        Node(
            package='nav2_behaviors', executable='behavior_server',
            name='behavior_server', output='screen', parameters=[configured],
            arguments=['--ros-args', '--log-level', 'behavior_server:=debug'],
            remappings=common_remaps,
        ),
        Node(
            package='nav2_bt_navigator', executable='bt_navigator',
            name='bt_navigator', output='screen',
            # Humble validates both default XML parameters while activating the
            # navigator, even though this deployment enables only
            # NavigateToPose. Point the unused legacy parameter at the same
            # known-good tree so the obsolete ThroughPoses tree is never read.
            parameters=[configured, {
                'default_nav_to_pose_bt_xml': bt_xml,
                'default_nav_through_poses_bt_xml': bt_xml,
            }],
            remappings=common_remaps,
        ),
        Node(
            package='nav2_velocity_smoother', executable='velocity_smoother',
            name='velocity_smoother', output='screen', parameters=[configured],
            remappings=common_remaps + [
                ('cmd_vel', '/cmd_vel_selected'),
                ('cmd_vel_smoothed', '/cmd_vel_smoothed'),
            ],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes,
                # Wi-Fi roaming briefly delayed local DDS bond callbacks by
                # more than Humble's 4 s default and caused a full Nav2
                # shutdown even though every process was still alive.
                'bond_timeout': 60.0,
            }],
        ),
    ])
