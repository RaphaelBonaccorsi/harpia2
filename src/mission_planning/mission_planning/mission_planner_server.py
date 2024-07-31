#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile

import sys, select
import math
import json
import time
import os
import psutil
from shutil import copyfile
from itertools import count
from std_srvs.srv import Empty

# Plansys 2 imports
#from plansys2_msgs.msg import CompletePlan
from plansys2_msgs.srv import GetProblem, GetPlan
from plansys2_msgs.action import ExecutePlan

from std_msgs.msg import String

from interfaces.srv import *
from interfaces.msg import *

def get_harpia_root_dir():
    """
    Returns the root directory of the Harpia project.

    This function returns the absolute path of the Harpia project's root directory. The root directory is assumed to be located in the current user's home directory under the name 'harpia'.

    Returns
    -------
    str
        The absolute path of the Harpia project's root directory.
    """
    return os.path.expanduser("~/harpia") # Check if this is the correct path ********

def parse_file_plan():
    """
    Parses the plan file and logs the plan.

    This function reads the plan.pddl file located in the 'pddl' directory of the Harpia project's root directory. It checks if a solution was found, calculates the CPU time and total time, and logs the plan to a new file in the 'results/plans' directory. The new file is named with an integer id, which is the smallest non-negative integer that is not already used as a file name in the directory.

    Returns
    -------
    tuple
        A tuple containing four elements:
        - success (bool): True if a solution was found, False otherwise.
        - cpu_time (float): The CPU time used to find the solution, or infinity if no solution was found.
        - total_time (float): The total time of the plan.
        - id (int): The id used as the name of the log file.
    """
    root_dir = get_harpia_root_dir()
    plan_path = os.path.join(root_dir, 'pddl/plan.pddl')

    success = False
    cpu_time = float('Inf')
    total_time = 0

    for line in open(plan_path):
        if 'Solution Found' in line: success = True
        if '; Time' in line:
            aux = line.split(' ')
            cpu_time = float(aux[-1].rstrip())
        if ': (' in line:
            aux = line.split(' ')
            # TODO: using `eval` is never very safe, change this to something more robust
            total_time += eval(aux[-1].rstrip())[0]

    # log path
    log_dir = os.path.join(root_dir, "results/plans/")

    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    ith_log_name = lambda i: os.path.join(log_dir, f"{i}.pddl")

    # Create an iterator over the unused log file names and use the next one available
    log_names = ((ith_log_name(i), i) for i in count(0) if not os.path.exists(ith_log_name(i)))
    log_path, id = next(log_names)

    copyfile(plan_path, log_path)
   
    return success, cpu_time, total_time, id

def log():
    """
    Logs the plan details to a JSON file.

    This function reads the mission log from the 'results/mission_log.json' file in the Harpia project's root directory. It parses the plan file to get the success status, CPU time, total time, and plan id. It then appends the CPU time and plan id to the last entry in the mission log and writes the updated log back to the file.

    """
    # log path
    root_dir = get_harpia_root_dir()
    log_path = os.path.join(root_dir, "results/mission_log.json")

    # open log
    with open(log_path, "r") as log_file:
        log_file = json.load(log_file)


    sucsses, cpu_time, total_time, plan_id = parse_file_plan()
    # add replan
    log_file[-1]['cpu_time'].append(cpu_time) 
    log_file[-1]['plans'].append(plan_id) 

    # dump log
    with open(log_path, 'w') as outfile:
        json.dump(log_file, outfile, indent=4)

class Plan(Node):
    """
    A ROS2 node that subscribes to the complete plan topic.

    This node subscribes to the 'plansys2/complete_plan' topic. It stores the complete plan and provides methods to check if the mission has ended.

    Attributes
    ----------
    sub : Subscription
        The subscription to the 'plansys2/complete_plan' topic.
    plan : CompletePlan
        The complete plan received from the 'plansys2/complete_plan' topic.
    """
    def __init__(self):
    """
        Initializes the Plan node and creates the action client.
        """
        super().__init__('plan')
        self.action_client = ActionClient(self, ExecutePlan, 'plansys2/execute_plan')
        self.current_goal = None

        # Send a request to get the current goal or plan
        self.send_goal_request()

    def plan_callback(self, data):
        """
        Callback function for the 'plansys2/complete_plan' subscription.

        This function is called when a new message is published on the 'plansys2/complete_plan' topic. It stores the received plan and destroys the subscription.

        Parameters
        ----------
        data : CompletePlan
            The complete plan received from the topic.
        """
        self.plan = data
        self.sub.destroy()  # Unsubscribe after receiving the complete plan

    def end_mission(self):
        """
        Checks if the mission has ended.

        This function checks if the id of the last action in the plan is equal to the id of the current action. If they are equal, it means that the mission has ended.

        Returns
        -------
        bool
            True if the mission has ended, False otherwise.
        """
        # Assuming there's a method to check if the mission has ended
        if not self.plan.plan:
            return False  # Handle empty plan case
        print(self.plan.plan[-1].action_id)
        # Implement logic to compare with current action if available
        return True  # Placeholder logic

    def unsubscribe(self):
        """
        Unsubscribes from the 'plansys2/complete_plan' topic.

        This function destroys the subscription to the 'plansys2/complete_plan' topic.
        """
        self.sub.destroy()

def mission_planning(self, req):
    """
     Executes the mission planning process.

    This function initializes a Plan object and checks if the mission has ended. If the mission has not ended, it generates a problem, calls the plan generator, logs the plan, and finally executes the plan. If any of these steps fail, it returns None. If all steps succeed, it returns True.
    
    Parameters
    ----------
    req : Request
        The request object containing the initial action id, and the drone's battery information including the discharge and recharge rates.

    Returns
    -------
    bool or None
        True if the mission planning process was successful, None if any step failed.
    """
    self.plan = Plan()
    self.has_plan = False
    if self.has_plan and self.plan.end_mission():
        return True

    self.get_logger().info("Creating problem")
    if not self.call_problem_generator():
        return None

    rclpy.sleep(0.1)  # Sleeps for 0.1 sec

    self.get_logger().info("Calling Plan generator")
    plan = self.call_plan_generator()
    if not plan:
        return None

    self.has_plan = True

    self.get_logger().info("Executing Plan")
    self.log()
    if not self.call_execute_plan(plan):
        return None

    return True
'''
    Callers for PLANSYS2 Services
'''

def try_call_srv(node, topic, srv_type):
    """
    Attempts to call a ROS service.

    This function waits for a ROS service to become available and then attempts to call it. If the service call succeeds, it returns True. If the service call fails, it logs the error and returns None.

    Parameters
    ----------
    node : rclpy.node.Node
        The ROS node to use for the service call.
    topic : str
        The name of the ROS service to call.
    srv_type : Type[rclpy.service.Service]
        The type of the ROS service.

    Returns
    -------
    bool or None
        True if the service call was successful, None if the service call failed.
    """
    cli = node.create_client(srv_type, topic)

    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    req = srv_type.Request()

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        return True
    else:
        node.get_logger().error('Service call failed %r' % (future.exception(),))
        return None

def call_problem_generator(node):
    return try_call_srv(node, 'problem_expert/get_problem', GetProblem)

def call_plan_generator(node):
    return try_call_srv(node, 'planner/get_plan', GetPlan)

# Substitui o parser e o dispatch
def call_execute_plan(node, plan):
    """
    Calls the execute plan action.

    Parameters
    ----------
    node : rclpy.node.Node
        The ROS node to use for the action call.
    plan : plansys2_msgs.msg.Plan
        The plan to execute.

    Returns
    -------
    bool or None
        True if the action call was successful, None if the action call failed.
    """
    action_client = ActionClient(node, ExecutePlan, 'execute_plan')
    if not action_client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error('Execute Plan action server not available')
        return None

    goal_msg = ExecutePlan.Goal()
    goal_msg.plan = plan

    future = action_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, future)
    goal_handle = future.result()

    if not goal_handle.accepted:
        node.get_logger().error('Execute Plan goal rejected')
        return None

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    result = result_future.result().result

    if result.success:
        return True
    else:
        node.get_logger().error('Execute Plan action failed')
        return None

# Example usage in a main function
def main(args=None):
    rclpy.init(args=args)
    node = Node('plansys2_client_node')

    if call_problem_generator(node):
        node.get_logger().info('Problem generated successfully')
    else:
        node.get_logger().error('Failed to generate problem')

    if call_plan_generator(node):
        node.get_logger().info('Plan generated successfully')
    else:
        node.get_logger().error('Failed to generate plan')

    # Assume `plan` is obtained from the plan generator
    plan = None  # Replace with actual plan
    if plan and call_execute_plan(node, plan):
        node.get_logger().info('Plan executed successfully')
    else:
        node.get_logger().error('Failed to execute plan')

    rclpy.shutdown()

if __name__ == '__main__':
    main()


def mission_planning_server():
    """
    Initializes a server for the mission planning service.

    This function initializes a ROS node named 'mission_planning_server' and creates a service named 'harpia/mission_planning' of type Empty. The service uses the mission_planning function to handle requests. After the service is created, the function logs a message indicating that the service is ready and then spins the node to keep it running.

    """
    rclpy.init()
    node = rclpy.create_node('mission_planning_server')
    srv = node.create_service(Empty, 'harpia/mission_planning', mission_planning)
    node.get_logger().info("Mission Planning Ready")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    mission_planning_server()
