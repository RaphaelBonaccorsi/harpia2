import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    harpia_home = LaunchConfiguration('harpia_home', default=os.getenv('HOME'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'harpia_home',
            default_value=os.getenv('HOME'),
            description='Defines the root directory of the project'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('mavros'), 'launch', 'px4.launch.py'
            )]),
            launch_arguments={
                'fcu_url': 'udp://:14540@localhost:14557',
                'log_output': 'log'
            }.items()
        ),

        Node(
            package='mission_planning',
            executable='mission_planner_server',
            name='mission_planner_as',
            output='screen'
        ),

        Node(
            package='mission_fault_mitigation',
            executable='mission_fault_mitigation_server',
            name='mission_fault_mitigation_server',
            output='screen'
        ),

        Node(
            package='mission_goal_manager',
            executable='mission_goal_manager_server',
            name='mission_goal_manager_server',
            output='screen',
            respawn=True
        ),

        Node(
            package='path_planning',
            executable='path_planning_server',
            name='path_planning_server',
            output='screen'
        ),

        Node(
            package='plansys2_problem_expert',
            executable='problem_expert',
            name='plansys2_problem_expert',
            output='screen',
            parameters=[{'domain_path': [harpia_home, '/pddl/domain.pddl']},
                        {'problem_path': [harpia_home, '/pddl/problem.pddl']}]
        ),

        Node(
            package='plansys2_planner',
            executable='planner',
            name='plansys2_planner',
            output='screen',
            parameters=[{'domain_path': [harpia_home, '/pddl/domain.pddl']},
                        {'problem_path': [harpia_home, '/pddl/problem.pddl']}]
        ),

        Node(
            package='plansys2_executor',
            executable='executor',
            name='plansys2_executor',
            output='screen',
            parameters=[{'domain_path': [harpia_home, '/pddl/domain.pddl']}]
        ),

        Node(
            package='plansys2_domain_expert',
            executable='domain_expert',
            name='plansys2_domain_expert',
            output='screen',
            parameters=[{'domain_path': [harpia_home, '/pddl/domain.pddl']}]
        ),
    ])
