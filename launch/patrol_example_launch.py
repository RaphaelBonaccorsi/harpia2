# Copyright 2019 Intelligent Robotics Lab
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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('route_executor2')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={'model_file': example_dir + '/pddl/domain.pddl'}.items()
    )
    # controller node
    control_cmd = Node(
        package='route_executor2',
        executable='planning_controller_node',
        name='planning_controller_node',
        output='screen',
        parameters=[]
    )


    # Specify the actions
    # move_cmd = Node(
    #     package='route_executor2',
    #     executable='move_action_node',
    #     name='move_action_node',
    #     output='screen',
    #     parameters=[]
    # )

    go_to_cmd = Node(
        package='route_executor2',
        executable='go_to.py',
        name='go_to',
        output='screen',
        parameters=[]
    )

    route_cmd = Node(
        package='route_executor2',
        executable='route_executor.py',
        name='route_executor',
        output='screen',
        parameters=[]
    )

    path_planner_cmd = Node(
        package='route_executor2',
        executable='path_planner.py',
        name='path_planner',
        output='screen',
        parameters=[]
    )

    

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(plansys2_cmd)
    ld.add_action(control_cmd)
    # ld.add_action(move_cmd)
    ld.add_action(go_to_cmd)
    
    ld.add_action(route_cmd)
    ld.add_action(path_planner_cmd)

    return ld
