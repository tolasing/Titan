import os 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

def generate_launch_description():
    map_file=os.path.join(get_package_share_directory("ros2_mapping"),"config","aic_lab.yaml")
    nav2_yaml=os.path.join(get_package_share_directory("ros2_mapping"),"config","amcl_config.yaml")

    map_server_node=Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time':False},
                    {'yaml_filename':map_file}]
    )

    amcl_node=Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_yaml]
    )

    map_lifecycle_node=Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time':False},
                    {'autostart':True},
                    {'node_names':['map_server','amcl']}]
    )

    return LaunchDescription(
      [map_server_node,
      amcl_node,
       map_lifecycle_node]
    )