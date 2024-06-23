import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml
from launch.actions import IncludeLaunchDescription, ExecuteProcess,RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml

def robosense_params(param_dir):
    param_substitutions = {
        'odom/lidar_topic': "/rslidar_points",
        'odom/lidar_frame': "rslidar",
        'odom/lidar_type': "robosense",
        # 'N_SCAN': "32",
        # 'Horizon_SCAN': "1800",
        # 'timeField': "time",
        # 'downsampleRate': "1",
        'odom/min_radius': "0.5",
        'odom/max_radius': "120.0"}

    configured_params = RewrittenYaml(
            source_file=param_dir,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True)
    return configured_params


def hesai_params(param_dir):
    param_substitutions = {
        'odom/lidar_topic': "/points_raw",
        'odom/lidar_frame': "hesai_lidar",
        'odom/lidar_type': "hesai",
        # 'N_SCAN': "16",
        # 'Horizon_SCAN': "1800",
        # 'timeField': "time",
        # 'downsampleRate': "1",
        'odom/min_radius': "0.5",
        'odom/max_radius': "120.0"}

    configured_params = RewrittenYaml(
            source_file=param_dir,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True)
    return configured_params

def velodyne_params(param_dir):
    param_substitutions = {
        'odom/lidar_topic': "/points_raw",
        'odom/lidar_frame': "velodyne",
        'odom/lidar_type': "velodyne",
        # 'N_SCAN': "16",
        # 'Horizon_SCAN': "1800",
        # 'timeField': "time",
        # 'downsampleRate': "1",
        'odom/min_radius': "0.5",
        'odom/max_radius': "120.0"}

    configured_params = RewrittenYaml(
            source_file=param_dir,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True)
    return configured_params


def livox_params(param_dir):
    param_substitutions = {
        'odom/lidar_topic': "/livox/lidar_custom",
        'odom/lidar_frame': "livox_frame",
        'odom/lidar_type': "livox",
        # 'N_SCAN': "4",
        # 'Horizon_SCAN': "6000",
        # 'timeField': "time",
        # 'downsampleRate': "1",
        'odom/min_radius': "0.3",
        'odom/max_radius': "40.0"}

    configured_params = RewrittenYaml(
            source_file=param_dir,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True)
    return configured_params

def launch_setup(context, *args, **kwargs):
    # Define the 'ig_lio' package directory
    
    # Define the path to your parameter file
    map_name = LaunchConfiguration('map_name').perform(context)
    map_location = LaunchConfiguration('map_location').perform(context)

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path').perform(context)
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    map_name = LaunchConfiguration('map_name').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    lidar_type = LaunchConfiguration('lidar_type').perform(context)
    config_path += "/" + robot_type +".yaml"
    print(lidar_type)
    # print(lidar_type)
    if lidar_type == "livox":
        config_path = livox_params(config_path)
    elif lidar_type == "hesai":
        config_path = hesai_params(config_path)
    elif lidar_type == "robosense":
        config_path = robosense_params(config_path)
    elif lidar_type == "velodyne":
        config_path = velodyne_params(config_path)
    map_dir = os.path.join(map_location, map_name)
    # reloc_param_path = os.path.join(
    #     ig_lio_reloc_dir,
    #     'params',
    #     reloc_param
    # )
    # lio_relocalization_node = Node(
    #     package='lio_relocalization',
    #     executable='lio_relocalization_node',
    #     name='lio_relocalization_node',
    #     output='screen',
    #     parameters=[reloc_param_path, {"map/map_name": LaunchConfiguration('map_name')},{'map/map_location': LaunchConfiguration('map_location')}],  # Pass the parameter file path directly
    # )
    # lio_tf_fusion_node =  Node(
    #     package='lio_relocalization',
    #     executable='lio_tf_fusion_node',
    #     name='lio_tf_fusion_node',
    #     output='screen',
    #     parameters=[reloc_param_path],  # Pass the parameter file path directly
    # )

    # ground_removal_node = Node(
    #     package="pointcloud_handler",
    #     executable="filter_pointcloud",
    #     name="ground_removal_node",
    #     respawn=True,
    #     output="screen",
    #     # arguments=['--ros-args', '--log-level', 'WARN'],
    #     parameters=[
    #         {"target_topic": "utlidar/cloud"},
    #         {"target_frame": "utlidar_lidar"},
    #         {"robot_frame": "trunk"},
    #         {"min_height": -0.3},
    #         {"max_height": 2.0},
    #         {"threshold": 0.1}
    #     ]
    # )
    
    print(config_path)
    # static_map_to_odom_node =  Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='world_to_map',
    #         arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'lio_odom']
    # )

    ig_lio_node = Node(
        package='ig_lio',
        executable='ig_lio_node',
        name='ig_lio_node',
        output='screen',
        parameters=[config_path]  # Pass the parameter file path directly
    )
    ig_lio_transform_node =  Node(
        package='ig_lio',
        executable='ig_lio_transform_node',
        name='ig_lio_transform_node',
        output='screen',
        parameters=[config_path]  # Pass the parameter file path directly
    )
    ig_lio_relocalize_node = Node(
        package='ig_lio',
        executable='ig_lio_relocalize_node',
        name='ig_lio_node',
        output='screen',
        parameters=[config_path, {'map/map_name': map_name}, {'map/map_location': map_location}]  # Pass the parameter file path directly
    )
    return [
        ig_lio_node,
        ig_lio_relocalize_node,
        ig_lio_transform_node
        # lio_tf_fusion_node,
    ]

def generate_launch_description():
    package_path = get_package_share_directory('ig_lio')
    default_config_path = os.path.join(package_path, 'config')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='sh',
        description='Name of the map to be used.')
    slam_type_arg = DeclareLaunchArgument(
        'slam_type',
        default_value='fast_lio',
        description='Slam type to be used.')
    map_location_arg = DeclareLaunchArgument(
        'map_location',
        default_value='',
        description='Map location')
    robot_type_arg = DeclareLaunchArgument(
        'robot_type', default_value='go2',
        description='robot_type'
    )
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_config_path_cmd,
        map_name_arg,
        slam_type_arg,
        map_location_arg,
        robot_type_arg,
        OpaqueFunction(function=launch_setup)
    ])
