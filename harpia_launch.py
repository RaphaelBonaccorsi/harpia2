import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    px4_path = LaunchConfiguration('px4_path', default='/home/harpia/PX4-Autopilot/build/px4_sitl_default/bin/px4')

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('px4_path', default_value='/home/harpia/PX4-Autopilot/build/px4_sitl_default/bin/px4', description='Path to the px4 executable'),

        # Execute PX4 process
        ExecuteProcess(
            cmd=[px4_path, 'udp://:14540@localhost:14557'],
            output='screen'
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
            executable='mission_goal_manager',
            name='mission_goal_manager',
            output='screen',
            parameters=[{'respawn': True}]
        ),
        Node(
            package='path_planning',
            executable='path_planning_server',
            name='path_planning_server',
            output='screen'
        ),
        Node(
            package='plansys2_problem_expert',
            executable='problem_expert_node',
            name='plansys2_problem_expert',
            output='screen',
            parameters=[{
                'domain_path': '/home/harpia/harpia_plansys2/pddl/domain.pddl',
                'problem_path': '/home/harpia/harpia_plansys2/pddl/problem.pddl'
            }]
        ),
        Node(
            package='plansys2_planner',
            executable='planner_node',
            name='plansys2_planner',
            output='screen',
            parameters=[{
                'domain_path': '/home/harpia/harpia_plansys2/pddl/domain.pddl',
                'problem_path': '/home/harpia/harpia_plansys2/pddl/problem.pddl'
            }]
        ),
        Node(
            package='plansys2_executor',
            executable='executor_node',
            name='plansys2_executor',
            output='screen',
            parameters=[{
                'domain_path': '/home/harpia/harpia_plansys2/pddl/domain.pddl'
            }]
        ),
        Node(
            package='plansys2_domain_expert',
            executable='domain_expert',
            name='plansys2_domain_expert',
            output='screen',
            parameters=[{
                'domain_path': '/home/harpia/harpia_plansys2/pddl/domain.pddl'
            }]
        )
    ])
