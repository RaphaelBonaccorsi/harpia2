from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mavros',
            executable='px4',
            name='px4',
            output='screen',
            parameters=[{
                'fcu_url': 'udp://:14540@localhost:14557',
                'log_output': 'log'
            }]
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
            executable='problem_expert',
            name='plansys2_problem_expert',
            output='screen',
            parameters=[{
                'domain_path': '~/harpia_plansys2/pddl/domain.pddl',
                'problem_path': '~/harpia_plansys2/pddl/problem.pddl'
            }]
        ),
        Node(
            package='plansys2_planner',
            executable='planner',
            name='plansys2_planner',
            output='screen',
            parameters=[{
                'domain_path': '~/harpia_plansys2/pddl/domain.pddl',
                'problem_path': '~/harpia_plansys2/pddl/problem.pddl'
            }]
        ),
        Node(
            package='plansys2_executor',
            executable='executor',
            name='plansys2_executor',
            output='screen',
            parameters=[{
                'domain_path': '~/harpia_plansys2/pddl/domain.pddl'
            }]
        ),
        Node(
            package='plansys2_domain_expert',
            executable='domain_expert',
            name='plansys2_domain_expert',
            output='screen',
            parameters=[{
                'domain_path': '~/harpia_plansys2/pddl/domain.pddl'
            }]
        )
    ])

