#!/usr/bin/env python3

# Pega latitude, longitude do GPS pelo MavROS
# Pega bateria pelo MavROS
# Controla waypoints
# Funções de calculo de distancia entre pontos
# Trabalha com o Plansys2 para replanejar missão

import rclpy #ROS2
#import actionlib
import sys
import math
import json
import time
import os
from rclpy.node import Node
from rclpy.action import ActionServer
import psutil
from std_srvs.srv import Empty
from rclpy.qos import QoSProfile

# Importação para mensagens do PlanSys2
from plansys2_msgs.srv import GetPlan, AddProblemGoal, GetProblem, ClearProblemKnowledge, RemoveProblemGoal, AddProblem, GetProblemGoal, GetStates
from plansys2_msgs.action import ExecutePlan
from plansys2_msgs.msg import Knowledge
from plansys2_msgs.srv import AffectParam

from diagnostic_msgs.msg import KeyValue

from std_msgs.msg import String

from interfaces.msg import ChangeMission, Mission, CompletePlan, GoalID
from interfaces.srv import PathPlanning
from interfaces.action import MissionPlanner


from mavros_msgs.msg import *
from mavros_msgs.srv import *

from sensor_msgs.msg import *
from geographic_msgs.msg import *
from geometry_msgs.msg import *

from action_msgs.msg import GoalStatusArray

## ----- Variáveis de controle -----

KB_UPDATE_ADD_KNOWLEDGE = 0
KB_UPDATE_RM_KNOWLEDGE = 2
KB_UPDATE_ADD_GOAL = 1
KB_UPDATE_RM_GOAL = 3
KB_UPDATE_ADD_METRIC = 4

KB_ITEM_INSTANCE = 0
KB_ITEM_FACT = 1
KB_ITEM_FUNCTION = 2
KB_ITEM_EXPRESSION = 3
KB_ITEM_INEQUALITY = 4

OP_UPDATE_KNOWLEDGE_BASE = 0
OP_REPLAN				 = 1
OP_ADD_RM_GOALS			 = 2

## ---------------------------------

def control_callback(data):
    """
    Callback function for stop the mission.

    Args:
        data (std_msgs.msg.String): ROS message containing control command information.

    Returns:
        None

    Note:
        need fix
    """
    ## que fix?

    # Assuming data is of type std_msgs/String
    if data.data == 'kill':
        rclpy.shutdown()

'''
	Classes to subscribe and publish services
'''

class Drone(Node):
    def __init__(self):
        super().__init__('drone_node')
        qos_profile = QoSProfile(depth=10)
        self.sub_position = self.create_subscription(NavSatFix, 'mavros/global_position/global', self.global_position_callback, qos_profile)
        self.sub_battery = self.create_subscription(BatteryState, 'mavros/battery', self.battery_state_callback, qos_profile)
        self.sub_mission = self.create_subscription(WaypointReached, 'mavros/mission/reached', self.reached_callback, qos_profile)
        self.latitude = None
        self.longitude = None
        self.battery = None
        self.current = None

    def global_position_callback(self, data):
        self.latitude = data.latitude
        self.longitude = data.longitude

    def battery_state_callback(self, data):
        self.battery = data.percentage * 100

    def reached_callback(self, data):
        self.current = data.wp_seq + 1

# Faz parte do actionlib: 
# In any large ROS based system, there are cases when someone would like to send a request to a node to perform some task,
# and also receive a reply to the request. This can currently be achieved via ROS services.
# In some cases, however, if the service takes a long time to execute,
# the user might want the ability to cancel the request during execution or get periodic feedback about how the request is progressing.
# The actionlib package provides tools to create servers that execute long-running goals that can be preempted. 
# It also provides a client interface in order to send requests to the server.

class MissionGoalManager(Node):
    def __init__(self):
        super().__init__('mission_goal_manager_server')
        qos_profile = QoSProfile(depth=10)
        self.a_server = ActionServer(
            self,
            MissionPlanner,
            'harpia/mission_goal_manager',
            execute_callback=self.execute_cb
        )
        self.uav = Drone()
        self.new_goals = None
        self.Mission_Sub = self.create_subscription(
            MissionPlanner.Goal,
            '/harpia/mission_goal_manager/goal',
            self.mission_callback,
            qos_profile
        )
        self.Goals_Sub = self.create_subscription(
            ChangeMission,
            '/harpia/ChangeMission',
            self.new_goal_callback,
            qos_profile
        )
        self.feedback_pub = self.create_publisher(String, '/harpia/mission_goal_manager/feedback', qos_profile)
        self.mission = None
        self.change_goals = None

    def mission_callback(self, data):
        self.mission = data.goal.mission

    def new_goal_callback(self, data):
        try:
            self.change_goals = data.op
            self.new_goals = data.goals
        except Exception as e:
            self.get_logger().error(f"Error in new_goal_callback: {e}")
            self.change_goals = 0

    def run(self):
        self.get_logger().info("Mission Goal Manager Service Ready")
        rclpy.spin(self)

    def register_cancel_callback(self, cancel_cb):
        self.a_server.set_aborted(MissionPlanner.Result())

    def abort(self):
        self.a_server.set_aborted(MissionPlanner.Result())

    def succeed(self):
        self.a_server.set_succeeded(MissionPlanner.Result())

    def execute_cb(self, goal_handle):
        self.get_logger().info(f"EXECUTE - MissionGoalManager with op {goal_handle.request.op}")

        if goal_handle.request.op == OP_UPDATE_KNOWLEDGE_BASE:
            update_knowledge_base(self, goal_handle.request.mission.uav, goal_handle.request.mission.map, goal_handle.request.mission.goals, "base_1")
        elif goal_handle.request.op == OP_REPLAN:
            self.get_logger().info("REPLAN - MissionGoalManager")
            replan(self, goal_handle.request.mission)
        elif goal_handle.request.op == OP_ADD_RM_GOALS:
            self.get_logger().error("ADD/RM GOALS - MissionGoalManager")
            replan(self, goal_handle.request.mission)
            return

        feedback_msg = String()
        feedback_msg.data = "Processing mission"
        self.feedback_pub.publish(feedback_msg)

        while not call_mission_planning(self) or self.change_goals:  # Passar `self` como argumento
            if self.change_goals:
                self.get_logger().info('MISSION GOAL Manager CANCEL')
                feedback_msg.data = 'Canceling mission goal'
                self.feedback_pub.publish(feedback_msg)
                goal_handle.publish_feedback(feedback_msg)
                op = self.change_goals
                self.change_goals = None
                replan(self, goal_handle.request.mission, self.uav, None, self.new_goals, op)
            elif not wait_until(lambda: self.uav.battery is not None, msg="Waiting for UAV battery..."):
                self.abort()
                return

            if self.uav.battery <= 20:
                base = go_to_base(goal_handle.request.mission, self.uav)
                replan(self, goal_handle.request.mission, self.uav, base, None, 0)
            else:
                replan(self, goal_handle.request.mission, self.uav, None, None, 0)
            feedback_msg.data = 'Mission ongoing'
            goal_handle.publish_feedback(feedback_msg)
            self.change_goals = 0

        land()
        time.sleep(30)
        self.succeed()



def wait_until(check, msg=None, rate=1):
    """
    Waits until a given condition is met.

    Args:
        check (callable): A function that returns a boolean. The function is called repeatedly until it returns True.
        msg (str, optional): A message to log while waiting. Defaults to None.
        rate (float, optional): Rate at which to check the condition in Hz. Defaults to 1.

    Returns:
        bool: True if the condition is met, False if ROS is shut down during waiting.

    Example:
        ```python
        # Wait until a condition is met with a log message
        result = wait_until(lambda: some_condition(), "Waiting for some_condition to be True...")

        # Wait until a condition is met with a specific rate
        result = wait_until(lambda: some_condition(), rate=2)
        ```

    """
    rate = rclpy.Rate(rate)

    while not check():
        if rclpy.ok():
            return False
        if msg is not None:
            rclpy.logging.info(msg)
        rate.sleep()

    return True

'''
	Callers for PLANSYS2 Services
    Services are another way that nodes can communicate with each other. 
    Services allow nodes to send a request and receive a response.
'''

def try_call_srv(node, topic, msg_ty=Empty):
    """
    Attempts to call a ROS2 service and returns True if successful, False otherwise.

    Args:
        node (Node): The ROS2 node instance.
        topic (str): The topic of the ROS2 service.
        msg_ty (Message, optional): The message type to be sent to the service. Defaults to Empty.

    Returns:
        bool: True if the service call is successful, False otherwise.
    """
    client = node.create_client(msg_ty, topic)

    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info(f'Service {topic} not available, waiting...')

    request = msg_ty.Request()
    future = client.call_async(request)

    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info('Service call successful')
        return True
    else:
        node.get_logger().info('Service call failed')
        return False

def call_problem_generator(node) -> bool:
    """
    Calls the ROS2 service to add a problem goal in PlanSys2.

    Args:
        node (Node): The ROS2 node instance.

    Returns:
        bool: True if the service call is successful, False otherwise.
    """
    return try_call_srv(node, 'problem_expert/add_problem_goal', AddProblemGoal)

def call_plan_generator(node) -> bool:
    """
    Calls the ROS2 service to generate a plan in PlanSys2.

    Args:
        node (Node): The ROS2 node instance.

    Returns:
        bool: True if the service call is successful, False otherwise.
    """
    return try_call_srv(node, 'planner/get_plan', Empty)

def call_parser(node) -> bool:
    """
    Calls the ROS2 service to get the current problem definition in PlanSys2.

    Args:
        node (Node): The ROS2 node instance.

    Returns:
        bool: True if the service call is successful, False otherwise.
    """
    return try_call_srv(node, 'problem_expert/get_problem', Empty)

def call_dispatch(node) -> bool:
    """
    Calls the ROS2 service to execute a plan in PlanSys2.

    Args:
        node (Node): The ROS2 node instance.

    Returns:
        bool: True if the service call is successful, False otherwise.
    """
    return try_call_srv(node, 'planner/execute_plan', ExecutePlan)

#def cancel_dispatch(node) -> bool:
    """
    Calls the ROS2 service to cancel plan execution in PlanSys2.

    Args:
        node (Node): The ROS2 node instance.

    Returns:
        bool: True if the service call is successful, False otherwise.
    """
    return try_call_srv(node, 'planner/cancel_execution', CancelExecution)

def call_mission_planning(node) -> bool:
    """
    Calls the ROS service for mission planning.

    Args:
        node (Node): The ROS2 node instance.

    Returns:
        bool: True if the service call is successful, False otherwise.
    """
    return try_call_srv(node, '/harpia/mission_planning', Empty)

'''
	Fuctions to calc the regios distances
'''

def calc_distances(regions):
	"""
    Calculate distances between regions.

    Args:
        regions (List[Region]): List of regions.

    Returns:
        List[Function]: List of functions representing distances between regions.
    """
	out = []
	for x in regions:
		for y in regions:
			if x == y: continue

			out.append(
				create_function(
					"distance",
					euclidean_distance(x.center.cartesian, y.center.cartesian),
					[KeyValue(key="region", value=x.name), KeyValue(key="region", value=y.name)]
				)
			)

	return out

def euclidean_distance(a, b):
	"""
    Calculate the Euclidean distance between two points.

    Args:
        a (Point): First point.
        b (Point): Second point.

    Returns:
        float: Euclidean distance between points.
    """
	return math.sqrt((b.x - a.x) ** 2 + (b.y - a.y) ** 2)

def geo_to_cart(geo_point, geo_home):
	"""
    Convert geographical coordinates to Cartesian coordinates.

    Args:
        geo_point (GeoPoint): Geographical coordinates.
        geo_home (GeoPoint): Home coordinates.

    Returns:
        Point: Cartesian coordinates.
    """

	def calc_y(lat, lat_):
		return (lat - lat_) * (10000000.0 / 90)

	def calc_x(longi, longi_, lat_):
		return (longi - longi_) * (
			6400000.0 * (math.cos(lat_ * math.pi / 180) * 2 * math.pi / 360)
		)

	x = calc_x(geo_point.longitude, geo_home.longitude, geo_home.latitude)
	y = calc_y(geo_point.latitude, geo_home.latitude)

	return Point(x, y, geo_point.altitude)

'''
	Functions to manipulate Knowledge base
'''

def try_update_knowledge(node, service_topic, service_type, request_data=None):
    """
    Try to update knowledge in the PlanSys2 knowledge base.

    Args:
        node (Node): The ROS2 node instance.
        service_topic (str): The service topic to call.
        service_type (Service): The type of the service.
        request_data (dict): The data to include in the service request.

    Returns:
        bool: True if the update was successful, False otherwise.
    """
    if request_data is None:
        request_data = {}
        
    client = node.create_client(service_type, service_topic)
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting...')

    request = service_type.Request()

    # Para ClearProblemKnowledge, você pode não precisar de request_data
    if request_data and hasattr(request, 'instances'):
        request.instances = request_data.instances
        request.predicates = request_data.predicates
        request.functions = request_data.functions
        request.goal = request_data.goal

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    success = future.result() is not None
    node.get_logger().info('Service call {}'.format('successful' if success else 'failed'))
    return success

def call_clear(node) -> bool:
    """
    Call the PlanSys2 service to clear all knowledge.

    Args:
        node (Node): The ROS2 node instance.

    Returns:
        bool: True if the service call was successful, False otherwise.
    """
    return try_update_knowledge(node, 'problem_expert/clear_problem_knowledge', ClearProblemKnowledge, None)

def add_instance(node, instance_name: str, instance_type: str) -> bool:
    """
    Adiciona uma instância ao conhecimento do PlanSys2.

    Args:
        node (Node): A instância do nó ROS2.
        instance_name (str): O nome da instância a ser adicionada.
        instance_type (str): O tipo da instância a ser adicionada.

    Returns:
        bool: True se a atualização foi bem-sucedida, False caso contrário.
    """
    client = node.create_client(AffectParam, 'problem_expert/add_problem_instance')
    
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting again...')
    
    request = AffectParam.Request()
    request.param.name = str(instance_name)
    request.param.type = instance_type

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None and future.result().success:
        node.get_logger().info('Instance added successfully.')
        return True
    else:
        node.get_logger().error(f'Failed to call service problem_expert/add_problem_instance: {future.result().error_info}')
        return False

def add_goal(node, item) -> bool:
    """
    Add a goal to the PlanSys2 knowledge base.

    Args:
        node (Node): The ROS2 node instance.
        item: The item to add.

    Returns:
        bool: True if the update was successful, False otherwise.
    """
    return try_update_knowledge(node, 'problem_expert/add_problem_goal', AddProblemGoal, item)

def remove_goal(node, item) -> bool:
    """
    Remove a goal from the PlanSys2 knowledge base.

    Args:
        node (Node): The ROS2 node instance.
        item: The item to remove.

    Returns:
        bool: True if the update was successful, False otherwise.
    """
    return try_update_knowledge(node, 'problem_expert/remove_problem_goal', RemoveProblemGoal, item)

def add_metric(node, item: str) -> bool:
    """
    Add a metric to the PlanSys2 knowledge base.

    Args:
        node (Node): The ROS2 node instance.
        item (str): The metric to add as a string.

    Returns:
        bool: True if the update was successful, False otherwise.
    """
    client = node.create_client(AddProblem, 'problem_expert/add_problem')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting again...')

    request = AddProblem.Request()
    request.problem = str(item) 

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None:
        node.get_logger().info('Metric added successfully.')
        return True
    else:
        node.get_logger().error('Failed to call service problem_expert/add_problem')
        return False

def get_knowledge(node, name, service_topic):
    """
    Get knowledge from the PlanSys2 knowledge base.

    Args:
        node (Node): The ROS2 node instance.
        name: The name of the knowledge item to retrieve.
        service_topic (str): The ROS2 service topic to call.

    Returns:
        KnowledgeItem: The knowledge item retrieved from the knowledge base.
    """
    client = node.create_client(GetProblem, service_topic)
    
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting...')

    request = GetProblem.Request()
    request.problem_name = name
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    result = future.result()
    if result is not None:
        return result.problem  
    else:
        node.get_logger().info('Failed to retrieve knowledge')
        return None

def get_function(node, function_name):
    """
    Get a function-like entity from the PlanSys2 knowledge base.

    Args:
        node (Node): The ROS2 node instance.
        function_name: The name of the function to retrieve.

    Returns:
        Problem: The function-like entity retrieved from the knowledge base.
    """
    client = node.create_client(GetProblem, 'problem_expert/get_problem')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting again...')

    request = GetProblem.Request()
    request.name = function_name

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None:
        node.get_logger().info('Function retrieved successfully.')
        return future.result().problem
    else:
        node.get_logger().error('Failed to call service problem_expert/get_problem')
        return None

def get_goal(node, goal_name):
    """
    Get a goal from the PlanSys2 knowledge base.

    Args:
        node (Node): The ROS2 node instance.
        goal_name: The name of the goal to retrieve.

    Returns:
        str: The goal PDDL retrieved from the knowledge base.
    """
    client = node.create_client(GetProblemGoal, 'problem_expert/get_problem_goal')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting again...')

    request = GetProblemGoal.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        return future.result().goal
    else:
        node.get_logger().error('Failed to call service problem_expert/get_problem_goal')
        return None
    
def get_predicate(node, predicate_name):
    """
    Get a predicate from the PlanSys2 knowledge base.

    Args:
        node (Node): The ROS2 node instance.
        predicate_name: The name of the predicate to retrieve.

    Returns:
        list: A list of predicates retrieved from the knowledge base.
    """
    client = node.create_client(GetStates, 'problem_expert/get_states')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting again...')

    request = GetStates.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        # Filter predicates by name
        predicates = [state for state in future.result().states if state.name == predicate_name]
        return predicates
    else:
        node.get_logger().error('Failed to call service problem_expert/get_states')
        return []

def create_object(item_name, item_type):
    """
    Create a knowledge item representing an object or instance.

    Args:
        item_name: The name of the object or instance.
        item_type: The type of the object or instance.

    Returns:
        Knowledge: The knowledge item representing the object or instance.
    """
    knowledge = Knowledge()
    knowledge.instances.append(f"{item_type}:{item_name}")

    return knowledge

def create_function(attribute_name, function_value, values=[]):
    """
    Create a knowledge item representing a function.

    Args:
        attribute_name: The name of the function attribute.
        function_value: The value of the function.
        values: List of KeyValue pairs representing the function arguments.

    Returns:
        Knowledge: The knowledge item representing the function.
    """
    knowledge = Knowledge()
    function_repr = f"{attribute_name}("
    function_repr += ", ".join([f"{kv.key}:{kv.value}" for kv in values])
    function_repr += f") = {function_value}"
    knowledge.functions.append(function_repr)

    return knowledge

def create_predicate(attribute_name, values=[], is_negative=False):
    """
    Create a knowledge item representing a predicate.

    Args:
        attribute_name: The name of the predicate attribute.
        values: List of KeyValue pairs representing the predicate arguments.
        is_negative: Boolean indicating whether the predicate is negative.

    Returns:
        Knowledge: The knowledge item representing the predicate.
    """
    knowledge = Knowledge()
    predicate_repr = f"{'not ' if is_negative else ''}{attribute_name}("
    predicate_repr += ", ".join([f"{kv.key}:{kv.value}" for kv in values])
    predicate_repr += ")"
    knowledge.predicates.append(predicate_repr)

    return knowledge

def create_metric(optimization, item):
    """
    Create a knowledge item representing a metric.

    Args:
        optimization: The optimization direction (e.g., minimize, maximize).
        item: The item for the metric.

    Returns:
        Knowledge: The knowledge item representing the metric.
    """
    knowledge = Knowledge()
    knowledge.goal = f"({optimization} ({item}))"

    return knowledge

def set_distances(node, map, goals, latitude, longitude):
    """
    Set distances in the knowledge base based on the current location.

    Args:
        node (Node): A instância do nó ROS2.
        map: The map information.
        goals: List of goals with regions.
        latitude: Current latitude.
        longitude: Current longitude.
    """
    geo_home = map.geo_home

    regions_name = [g.region for g in goals]

    # Get current latitude and longitude
    cart_location = geo_to_cart(GeoPoint(latitude, longitude, 15), geo_home)

    for base in map.bases:
        d = euclidean_distance(base.center.cartesian, cart_location)
        obj = create_function(
            "distance",
            round(d, 2),
            [KeyValue("region", "aux"), KeyValue("region", base.name)]
        )
        add_instance(node, obj, "function")

    for region in map.roi:
        if region.name in regions_name:
            d = euclidean_distance(cart_location, region.center.cartesian)
            obj = create_function(
                "distance",
                round(d, 2),
                [KeyValue("region", "aux"), KeyValue("region", region.name)]
            )
            add_instance(node, obj, "function")

def regions_to_perform_action(goals, action):
    """
    Get regions associated with a specific action.

    Args:
        goals: List of goals with regions.
        action: Action to filter regions.

    Returns:
        List of regions associated with the specified action.
    """
    return [step.region for step in goals if step.action == action]

def update_knowledge_base(node, uav, map, goals, at):
    """
    Update the knowledge base with information related to the UAV, map, and goals.

    Args:
        node (Node): A instância do nó ROS2.
        uav (UAV): UAV object containing information about the drone.
        map (Map): Map object containing information about the environment.
        goals (List[Goal]): List of goals to be achieved by the UAV.
        at (str): The initial location of the UAV.

    Returns:
        None
    """
    regions = map.roi
    bases = map.bases
    pulverize = regions_to_perform_action(goals, 'pulverize')
    photo = regions_to_perform_action(goals, 'take_picture')
    end = regions_to_perform_action(goals, 'end')

    # In this version, goals are only added at the initial location
    # To prevent repetition, a set is used to store unique regions in goals
    goals_regions = set(pulverize + photo)

    regions_obj = [r for r in regions if r.name in goals_regions] + bases

    # Clear the knowledge base
    call_clear(node)

    # Set drone initial state

    # Adding objects to knowledge base
    add_instance(node, "at", "predicate")
    add_instance(node, create_function("battery-capacity", uav.battery.capacity), "function")
    add_instance(node, create_function("velocity", uav.frame.efficient_velocity), "function")
    add_instance(node, create_function("battery-amount", 100), "function")
    add_instance(node, create_function("discharge-rate-battery", uav.battery.discharge_rate), "function")
    add_instance(node, create_function("input-amount", 0), "function")
    add_instance(node, create_function("mission-length", 0), "function")
    add_instance(node, create_function("input-capacity", uav.input_capacity), "function")

    for r in regions:
        add_instance(node, str(r.name), "region")

    for b in bases:
        add_instance(node, str(b.name), "base")

    for fact in calc_distances(regions_obj):
        add_instance(node, str(fact.instances), "fact")

    total_goals = 0

    for i in pulverize:
        add_instance(node, create_predicate("pulverize-goal", [KeyValue(key="region", value=i)]), "predicate")
        add_goal(node, create_predicate("pulverized", [KeyValue(key="region", value=i)]))
        total_goals += 1

    for i in photo:
        add_instance(node, create_predicate("picture-goal", [ KeyValue(key="region", value=i)]), "predicate")
        add_goal(node, create_predicate("taken-image", [ KeyValue(key="region", value=i)]))
        total_goals += 1


    for i in end:
        add_goal(node, create_predicate("at", [KeyValue(key="region", value=i)]))

    add_metric(node, create_metric("minimize (mission-length)", []))

def find_at(map, goals, latitude, longitude):
    """
    Finds a base that is within a 50m radius. If none is found, try to find a region which has a center
    within the same range. If none is found, return `None`. If there is one of these waypoints in range,
    the drone is considered at that waypoint.

    Args:
        map (Map): Map object containing information about the environment.
        goals (List[Goal]): List of goals to be achieved by the UAV.
        latitude (float): Current latitude of the UAV.
        longitude (float): Current longitude of the UAV.

    Returns:
        Union[Base, Region, None]: Returns a Base or Region object if found within a 50m radius,
        otherwise returns None.
    """
    # Get current latitude and longitude
    cart_location = geo_to_cart(GeoPoint(latitude, longitude, 15), map.geo_home)

    # Check if any base is within a 50m radius
    for base in map.bases:
        d = euclidean_distance(base.center.cartesian, cart_location)
        if d <= 50:
            return base

    # Get region names from goals
    region_names = {g.region for g in goals}

    # Check if any region has a center within a 50m radius
    for region in map.roi:
        if region.name in region_names:
            d = euclidean_distance(cart_location, region.center.cartesian)
            if d <= 50:
                return region

    return None


def remove_instance(node, item) -> bool:
    """
    Remove an instance from the PlanSys2 knowledge base.

    Args:
        node (Node): The ROS2 node instance.
        item: The item to remove.

    Returns:
        bool: True if the update was successful, False otherwise.
    """
    client = node.create_client(ClearProblemKnowledge, 'problem_expert/clear_knowledge')
    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error('Service not available')
        return False

    request = ClearProblemKnowledge.Request()
    # Adapt request based on service definition, e.g.,
    # request.knowledge = item
    # Make sure the service request is constructed correctly
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()

    if response is not None:
        return True
    else:
        node.get_logger().error('Service call failed')
        return False
    
def find_nearest_base(map, latitude, longitude):
    """
    Finds the nearest base to the given latitude and longitude.

    Args:
        map (Map): Map object containing information about the environment.
        latitude (float): Latitude of the location.
        longitude (float): Longitude of the location.

    Returns:
        Base: Nearest Base object.
    """
    # Get current latitude and longitude
    cart_location = geo_to_cart(GeoPoint(latitude, longitude, 15), map.geo_home)

    # Find the nearest base using euclidean distance
    return min(map.bases, key=lambda base: euclidean_distance(base.center.cartesian, cart_location))

def call_path_planning(r_from, r_to, map):
    """
    Calls the path planning service to calculate the path from one region to another.

    Args:
        r_from (Region): Starting region.
        r_to (Region): Destination region.
        map (Map): Map object containing information about the environment.

    Returns:
        PathPlanning.Response or None: Response from the path planning service or None if the service call fails.
    """
    try:
        path_planning_client = self.create_client(PathPlanning, '/harpia/path_planning')
        while not path_planning_client.wait_for_service(timeout_sec=1.0):
            print('Service not available, waiting...')
        request = PathPlanning.Request()
        request.r_from = r_from.center
        request.r_to = r_to.center
        request.r_from_name = r_from.name
        request.r_to_name = r_to.name
        request.map = map
        future = path_planning_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result()
        else:
            print('Service call failed')
            return None
    except Exception as e:
        print('Service call failed:', str(e))
        return None

    # Need to test this function
def replan(node, mission, uav, base, goals, op):
    """
    Replans the mission based on the given operation.

    Args:
        node (Node): A instância do nó ROS2.
        mission (Mission): Mission object containing information about the mission.
        uav (Drone): UAV object containing information about the UAV.
        base (Base or None): Base object if the operation is to return home, else None.
        goals (list): List of goals to be added or removed from the mission.
        op (int): Operation code (0: continue mission, 1: add goals, 2: remove goals).

    Returns:
        None
    """
    rclpy.logging.get_logger().info("CANCEL DISPATCH - MissionGoalManager")

    if not wait_until(lambda: mission.goals != [], msg="Waiting for Regions..."):
        return
    if not wait_until(lambda: uav.latitude is not None, msg="Waiting for position..."):
        return

    if goals != None:
        pulverize = regions_to_perform_action(goals, 'pulverize')
        photo = regions_to_perform_action(goals, 'take_picture')

    if base is None:
        # Here we want to continue the mission.
        at = get_predicate("at")

        if len(at.attributes) == 0:
            base_or_region_nearby = find_at(mission.map, mission.goals, uav.latitude, uav.longitude)
            if base_or_region_nearby is not None:
                rclpy.logging.get_logger().info(f'at = {base_or_region_nearby}')
                at = create_predicate("at", [KeyValue("region", base_or_region_nearby.name)])
            else:
                add_instance(node, "aux", "region")
                add_instance(node, create_predicate("its-not-base", [KeyValue("region", "aux")]), "predicate")
                set_distances(node, mission.map, mission.goals, uav.latitude, uav.longitude)

                at = create_predicate("at", [KeyValue("region", "aux")])
        else:
            at = at.attributes[0]
    else:
        at_move = get_predicate("at-move")
        if len(at_move.attributes) > 0:
            remove_instance(node, at_move.attributes[0])

        at_kb = get_predicate("at")
        if len(at_kb.attributes) > 0:
            remove_instance(node, at_kb.attributes[0])

        rclpy.logging.get_logger().info(f"base = {base}")
        at = create_predicate("at", [KeyValue("base", base.name)])

    add_instance(node, at)

    bat = get_function("battery-amount")
    remove_instance(node, bat.attributes[0])

    if not wait_until(lambda: uav.battery is not None, msg="Waiting for UAV battery..."):
        return

    obj = create_function("battery-amount", uav.battery)
    add_instance(node, obj)

    has_end = False
    if op == 0:
        for goal in get_goal('').attributes:
            if goal.attribute_name != "at":
                for goal_achived in get_predicate(goal.attribute_name).attributes:
                    if goal.values[0].value == goal_achived.values[0].value:
                        remove_goal(node, goal)
            else:
                has_end = True

    elif op == 1:
        if pulverize:
            add_instance(node, create_predicate("has-pulverize-goal"), "predicate")

        if photo:
            add_instance(node, create_predicate("has-picture-goal"), "predicate")

        for i in pulverize:
            add_instance(node, create_predicate("pulverize-goal", [KeyValue("region", i)]), "predicate")
            add_instance(node, create_function("pulverize-path-len", 314, [KeyValue("region", i)]), "function")
            add_goal(node, create_predicate("pulverized", [KeyValue("region", i)]))

        for i in photo:
            add_instance(node, create_predicate("picture-goal", [KeyValue("region", i)]), "predicate")
            add_instance(node, create_function("picture-path-len", 1000, [KeyValue("region", i)]), "function")
            add_goal(node, create_predicate("taken-image", [KeyValue("region", i)]))

        for g in goals:
            mission.goals.append(g)
        regions = mission.map.roi
        pulverize = regions_to_perform_action(mission.goals, 'pulverize')
        photo = regions_to_perform_action(mission.goals, 'take_picture')

        goals_regions = set(pulverize + photo)
        bases = mission.map.bases
        regions_obj = [r for r in regions if r.name in goals_regions] + bases

        for fact in calc_distances(regions_obj):
            add_instance(node, fact)

    elif op == 2:
        for goal in get_goal('').attributes:
            if goal.attribute_name == 'taken-image':
                for goal_toRemove in photo:
                    if goal.values[0].value == goal_toRemove:
                        remove_goal(node, goal)
            elif goal.attribute_name == 'pulverized':
                for goal_toRemove in pulverize:
                    if goal.values[0].value == goal_toRemove:
                        remove_goal(node, goal)
            else:
                print('not implemented yet')

        add_instance(node, obj)

def go_to_base(mission, uav):
    """
    Go to the nearest base immediately and land.

    Args:
        mission (Mission): Mission object containing information about the mission.
        uav (Drone): UAV object containing information about the UAV.

    Returns:
        Base or None: Base object if the drone successfully went to the nearest base, else None.
    """

    rate = rclpy.Rate(1)

    if not wait_until(lambda: uav.latitude is not None, msg="Waiting for position..."):
        rclpy.logerr("CRITICAL - MissionGoalManager was terminated while going to nearest base and will not finish the action")
        return

    at = find_at(mission.map, mission.goals, uav.latitude, uav.longitude)
    base = find_nearest_base(mission.map, uav.latitude, uav.longitude)

    route = call_path_planning(at, base, mission.map)

    # send route to uav
    clear_mission()
    rclpy.loginfo("Send Route")
    send_route(route.waypoints)

    # set mode to mission
    rclpy.loginfo("Set Mode")
    set_mode("AUTO.MISSION")

    # wait to arrive.
    uav.current = 0
    while uav.current < len(route.waypoints.waypoints):
        rclpy.sleep(1)

    # land
    land()
    rclpy.sleep(30)

    return base

'''
	Callers for MAVRos Services
'''

def mavros_cmd(topic, msg_ty, error_msg="MAVROS command failed: ", **kwargs):
    """
    Send a command to MAVROS service.

    Args:
        topic (str): ROS service topic for MAVROS command.
        msg_ty (Message Type): ROS message type for the command.
        error_msg (str, optional): Error message to log in case of failure. Defaults to "MAVROS command failed: ".
        **kwargs: Additional keyword arguments for the command.
    """
    try:
        service_client = node.create_client(msg_ty, topic)
        while not service_client.wait_for_service(timeout_sec=1.0):
            print(f"Waiting for service {topic}...")
        request = msg_ty.Request()
        for key, value in kwargs.items():
            setattr(request, key, value)
        future = service_client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        if future.result() is not None:
            response = future.result()
            print(response)
        else:
            print(f"{error_msg} Service call failed.")
    except Exception as e:
        print(f"{error_msg} {e}")

def land():
    """
    Command the drone to land using MAVROS.
    """
    mavros_cmd(
        '/mavros/cmd/land',
        CommandTOL,
        error_msg="Landing failed",
        altitude=10, latitude=0, longitude=0, min_pitch=0, yaw=0
    )


def set_mode(mode):
    """
    Set the flight mode of the drone using MAVROS.
    """
    mavros_cmd(
        '/mavros/set_mode',
        SetMode,
        error_msg="Set mode failed",
        custom_mode=mode
    )


def clear_mission():
    """
    Clear the current mission in MAVROS.
    """
    mavros_cmd(
        '/mavros/mission/clear',
        WaypointClear,
        error_msg="Clear mission failed"
    )


def send_route(route):
    """
    Send a route (list of waypoints) to the MAVROS mission planner.
    """
    mavros_cmd(
        '/mavros/mission/push',
        WaypointPush,
        error_msg="Send route failed",
        start_index=route.current_seq,
        waypoints=route.waypoints
    )

def main(args=None):
    rclpy.init()
    global node
    node = MissionGoalManager()
    node.run()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
