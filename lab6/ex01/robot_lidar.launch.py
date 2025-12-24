# Copyright 2022 Open Source Robotics Foundation, Inc.
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.actions import ExecuteProcess


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('robot_bringup_lid')
    pkg_project_description = get_package_share_directory('robot_description_lid')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # # Load the URDF file from "description" package
    # urdf_file  =  os.path.join(pkg_project_description, 'urdf', 'robot.urdf')
    # with open(urdf_file, 'r') as infp:
    #     robot_desc = infp.read()

    urdf_path  =  os.path.join(pkg_project_description, 'urdf', 'my_robot_lidar.urdf.xacro')
    robot_desc = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    world_path = os.path.join(pkg_project_bringup, 'worlds', 'my_lidar_world.sdf')
    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
      #  launch_arguments={'gz_args': f"-r {world_path}"}.items(),
        launch_arguments={
          "gz_args": f"-r gpu_lidar_sensor.sdf"}.items(),


    )

    # Spawn robot
    create = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'robot',
                '-topic', 'robot_description',
                '-x', '-4.0',
                '-z', '0.5',
                '-world', 'gpu_lidar_sensor'],

        output='screen',
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'robot_description': robot_desc},
            {'frame_prefix': "robot/"}
        ]
    )

    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'diff_drive.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'robot_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    lidar_stop = Node(
                package='lidar_moving',
                executable='lidar_moving',
                name='lidar_moving_node',
                output='screen',
            )
    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        bridge,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        rviz,
        TimerAction(
            period=5.0,
             actions=[create]),
    ])


