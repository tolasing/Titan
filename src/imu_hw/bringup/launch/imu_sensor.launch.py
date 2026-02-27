# Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#
# Authors: Subhas Das, Denis Stogl
import os 
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.actions import IncludeLaunchDescription,TimerAction
import xacro

def generate_launch_description():
    robot_description=Command(['ros2 param get --hide-type /my_robot_state_publisher_node robot_description'])
    controller_config=os.path.join(
        get_package_share_directory
        ("imu_interface"),"config","imu_sensor_config.yaml"
    )

    controller_ros2_control= Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description':robot_description},controller_config],
        output={
        'stdout':'screen',
        'stderr':'screen'
        },
        )
    
    delayed_controller_ros2_control=TimerAction(period=3.0,actions=[controller_ros2_control])
    

    # add the spawner node for the fts_broadcaster
    fts_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fts_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    imu_spawner=Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_broadcaster"]
    )

    delayed_imu_spawner=RegisterEventHandler(
        event_handler=OnProcessStart(
        target_action=controller_ros2_control,
        on_start=[imu_spawner]
        )
    )

    Joint_state_spawner=Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"]
    )

    delayed_Joint_state_spawner=RegisterEventHandler(
        event_handler=OnProcessStart(target_action=
                                     controller_ros2_control,
                                     on_start=[Joint_state_spawner])
        )

    return LaunchDescription([
       # delayed_controller_ros2_control,
        #delayed_imu_spawner,
        #delayed_Joint_state_spawner,
        imu_spawner
    ])