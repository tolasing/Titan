import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node 
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.actions import IncludeLaunchDescription,TimerAction

def generate_launch_description():
    #Data Input
    xacro_file='plane.xacro'
    package_name="plane"

    print("fetching the xacro")
    #specifying  the xacro file path
    #xacro_file_path=os.path.join(get_package_share_directory(package_name)
    #,"urdf",xacro_file)
    xacro_file_path=os.path.join(get_package_share_directory(package_name)
    ,"urdf",xacro_file)

    #initializing the robot_state_publisher_node
    robot_state_publisher_node=Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='plane_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time':False,
        'robot_description':Command(['xacro ',xacro_file_path])}],
        output="screen"
    )

    #specifying the configuration for RVIZ
    rviz_config=os.path.join(get_package_share_directory(package_name),'rviz','test_encoder.rviz')

    rviz_node=Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rivz_node',
        parameters=[{'use_sim_time':False}],
        arguments=['-d',rviz_config])
    
    #create and return launch description object

    return LaunchDescription(
        [
            robot_state_publisher_node,
            #rviz_node,
        ]
    )