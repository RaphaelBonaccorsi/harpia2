import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    harpia_home = LaunchConfiguration('harpia_home', default=os.environ['HOME'])

    mavros_launch_file = os.path.join(
        get_package_share_directory('mavros'),
        'launch',
        'px4.launch'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'harpia_home',
            default_value=os.environ['HOME'],
            description='Defines the root directory of the project'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mavros_launch_file),
            launch_arguments={
                'fcu_url': 'udp://:14540@localhost:14557',
                'log_output': 'log'
            }.items()
        ),

        Node(
            package='mission_planning',
            executable='mission_planner_server.py',
            name='mission_planner_as',
            output='screen'
        ),

        Node(
            package='mission_fault_mitigation',
            executable='mission_fault_mitigation_server.py',
            name='mission_fault_mitigation_server',
            output='screen'
        ),

        Node(
            package='mission_goal_manager',
            executable='mission_goal_manager_server.py',
            name='mission_goal_manager_server',
            output='screen',
            respawn=True
        ),

        Node(
            package='path_planning',
            executable='path_planning_server.py',
            name='path_planning_server',
            output='screen'
        )

        # Uncomment the following lines to add more nodes or includes as needed
        # Node(
        #     package='fault_detection',
        #     executable='weather_check_server.py',
        #     name='weather_check_server',
        #     output='screen'
        # ),
        # Node(
        #     package='fault_detection',
        #     executable='DroneInfoCompiler',
        #     name='drone_info',
        #     output='screen'
        # ),
        # Node(
        #     package='fault_detection',
        #     executable='anomaly_detector.py',
        #     name='anomaly_detector',
        #     output='screen'
        # ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(
        #         get_package_share_directory('rosplan_planning_system'),
        #         'launch',
        #         'interfaced_planning_system.launch.py'
        #     )),
        #     launch_arguments={
        #         'domain_path': os.path.join(harpia_home, 'pddl', 'domain.pddl'),
        #         'autom_gen_problem_path': os.path.join(harpia_home, 'pddl', 'problem.pddl'),
        #         'data_path': os.path.join(harpia_home, 'pddl')
        #     }.items()
        # ),
        # Node(
        #     package='rosplan_interface_harpia',
        #     executable='RPHarpiaExecutor',
        #     name='rosplan_interface_recharge_input',
        #     output='screen',
        #     parameters=[{
        #         'knowledge_base': 'rosplan_knowledge_base',
        #         'pddl_action_name': 'recharge_input',
        #         'action_dispatch_topic': '/rosplan_plan_dispatcher/action_dispatch',
        #         'action_feedback_topic': '/rosplan_plan_dispatcher/action_feedback'
        #     }]
        # ),
        # Node(
        #     package='rosplan_interface_harpia',
        #     executable='RPHarpiaExecutor',
        #     name='rosplan_interface_go_to',
        #     output='screen',
        #     parameters=[{
        #         'knowledge_base': 'rosplan_knowledge_base',
        #         'pddl_action_name': 'go_to',
        #         'action_dispatch_topic': '/rosplan_plan_dispatcher/action_dispatch',
        #         'action_feedback_topic': '/rosplan_plan_dispatcher/action_feedback'
        #     }]
        # ),
        # Node(
        #     package='rosplan_interface_harpia',
        #     executable='RPHarpiaExecutor',
        #     name='rosplan_interface_pulverize_region',
        #     output='screen',
        #     parameters=[{
        #         'knowledge_base': 'rosplan_knowledge_base',
        #         'pddl_action_name': 'pulverize_region',
        #         'action_dispatch_topic': '/rosplan_plan_dispatcher/action_dispatch',
        #         'action_feedback_topic': '/rosplan_plan_dispatcher/action_feedback'
        #     }]
        # ),
        # Node(
        #     package='rosplan_interface_harpia',
        #     executable='RPHarpiaExecutor',
        #     name='rosplan_interface_take_img',
        #     output='screen',
        #     parameters=[{
        #         'knowledge_base': 'rosplan_knowledge_base',
        #         'pddl_action_name': 'take_image',
        #         'action_dispatch_topic': '/rosplan_plan_dispatcher/action_dispatch',
        #         'action_feedback_topic': '/rosplan_plan_dispatcher/action_feedback'
        #     }]
        # ),
        # Node(
        #     package='rosplan_interface_harpia',
        #     executable='RPHarpiaExecutor',
        #     name='rosplan_interface_recharge_battery',
        #     output='screen',
        #     parameters=[{
        #         'knowledge_base': 'rosplan_knowledge_base',
        #         'pddl_action_name': 'recharge_battery',
        #         'action_dispatch_topic': '/rosplan_plan_dispatcher/action_dispatch',
        #         'action_feedback_topic': '/rosplan_plan_dispatcher/action_feedback'
        #     }]
        # )
    ])

if __name__ == '__main__':
    generate_launch_description()
