from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to launch RViz'
    )
    input_points_arg = DeclareLaunchArgument(
        'input_points',
        default_value='/off_highway_premium_radar_sample_driver/locations',
        description='PointCloud2 topic for radar points'
    )
    rcs_field_arg = DeclareLaunchArgument(
        'rcs_field',
        default_value='radar_cross_section',
        description='PointCloud2 field name for RCS'
    )
    power_field_arg = DeclareLaunchArgument(
        'power_field',
        default_value='power',
        description='PointCloud2 field name for power (fallback if RCS missing)'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (set true when playing bags with --clock)'
    )

    # Get package share directory
    pkg_share = FindPackageShare(package='radar4motion').find('radar4motion')
    
    # Create node with parameters
    radar4motion_node = Node(
        package='radar4motion',
        executable='radar4motion',
        name='radar4motion',
        output='screen',
        parameters=[
            # INI paths
            {'str_odom_ini_path': PathJoinSubstitution([pkg_share, 'config', 'radar_point_cloud_odometry_online.ini'])},
            {'str_radar_ego_motion_ini_path': PathJoinSubstitution([pkg_share, 'config', 'radar_ego_motion_estimation.ini'])},
            
            # Offline version (VoD Dataset)
            {'b_flag_run_vod_dataset': False},
            {'str_vod_dataset_base_path': PathJoinSubstitution([pkg_share, 'dataset'])},
            {'scenario_num': 1},
            
            # Set virtual frame rate for sequentially reading VoD dataset
            {'i_vod_frame_rate': 100},
            
            # Set eval result file path
            {'vod_eval_folder_path': PathJoinSubstitution([pkg_share, 'test'])},
            
            # Online version (ROS topic)
            {'s_input_points_': LaunchConfiguration('input_points')},
            
            {'str_origin_': 'Map'},
            {'str_base_link_': 'radar'},
            
            {'str_rcs_field_name_': LaunchConfiguration('rcs_field')},
            {'str_power_field_name_': LaunchConfiguration('power_field')},
            
            {'b_accum_whole_point_clouds_': True},
            
            {'d_map_voxel_leaf_size_': 0.5},
            
            {'min_static_points_num': 15},

            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'radar_odom.rviz'])],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_arg,
        input_points_arg,
        rcs_field_arg,
        power_field_arg,
        use_sim_time_arg,
        radar4motion_node,
        GroupAction([rviz_node]),
    ])
