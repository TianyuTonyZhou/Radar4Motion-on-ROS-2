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
            {'str_odom_ini_path': PathJoinSubstitution([pkg_share, 'config', 'radar_point_cloud_odometry.ini'])},
            {'str_radar_ego_motion_ini_path': PathJoinSubstitution([pkg_share, 'config', 'radar_ego_motion_estimation.ini'])},
            
            # Offline version (VoD Dataset)
            {'b_flag_run_vod_dataset': True},
            {'str_radar4motion_path': pkg_share},
            {'str_vod_dataset_base_path': '/home/ailab/AILabDataset/01_Open_Dataset/19_ViewofDelft/'},
            {'scenario_num': 3},
            
            # Set virtual frame rate for sequentially reading VoD dataset
            {'i_vod_frame_rate': 100},
            
            # Set eval result file path
            {'vod_eval_folder_path': PathJoinSubstitution([pkg_share, 'test'])},
            
            # Online version (ROS topic)
            {'s_input_points_': '/afi910_cloud_node/cloud'},
            {'str_origin_': 'Map'},
            {'str_base_link_': 'radar'},
            
            {'str_rcs_field_name_': 'RCS'},
            {'str_power_field_name_': 'power'},
            {'b_accum_whole_point_clouds_': True},
            {'d_map_voxel_leaf_size_': 0.5},
            {'min_static_points_num': 15},
        ]
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'radar_odom.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_arg,
        radar4motion_node,
        GroupAction([rviz_node]),
    ])
