#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, TransitionEvent

class LifecycleMannager(Node):
    def __init__(self):
        super().__init__('lifecycle_controller')

        self.nodes = {}
        self.wait_for_nodes_timer = None

        self.activating_nodes = False
        self.activating_nods_timer = None

    def add_node(self, node_name, depends_on):
        self.get_logger().info(f'Adding node {node_name}')

        sub = self.create_subscription(
            TransitionEvent,
            f'{node_name}/transition_event',
            lambda msg: self.transition_event_callback(node_name, msg),
            10)
        
        self.nodes[node_name] = {
            'depends_on': depends_on,
            'state': None,
            'sub': sub,
        }

        tries = 0
        def check_service_availability(node_name, timer, client):
            nonlocal tries
            tries += 1
            if client.service_is_ready():
                self.get_logger().info(f'{node_name} OK')
                timer.cancel()  # Stop the timer
                
                # Create a request
                def handle_get_state_response(future):
                    response = future.result()
                    if response is not None:
                        state = response.current_state.label
                        self.get_logger().info(f'get state: {node_name} -> {state}')
                        self.nodes[node_name]['state'] = state

                request = GetState.Request()
                future = client.call_async(request)
                future.add_done_callback(handle_get_state_response)

            elif tries > 20:
                self.get_logger().warn(f'Could not find lifecycle node {node_name}.')
                timer.cancel()
                del self.nodes[node_name]


        client = self.create_client(GetState, f'/{node_name}/get_state')
        timer = self.create_timer(0.5, lambda: check_service_availability(node_name, timer, client))
        
    def wait_for_nodes_and_start(self):

        if self.wait_for_nodes_timer is None:

            def wait_for_nodes():
                self.get_logger().info(f'Checking nodes...')

                if all([node['state'] is not None for node in self.nodes.values()]):
                    self.wait_for_nodes_timer.cancel()
                    self.wait_for_nodes_timer = None
                    self.get_logger().info('All nodes are running, starting active nodes...')
                    self.start_nodes_activation()

            self.wait_for_nodes_timer = self.create_timer(5, wait_for_nodes)
        else:
            self.get_logger().warn('wait_for_nodes_timer already exists')

    def start_nodes_activation(self):
        self.activating_nodes = True

        if self.activating_nods_timer is not None:
            self.activating_nods_timer.cancel()

        self.activating_nods_timer = self.create_timer(0.5, self.configure_unconfigured_nodes)

    def stop_nodes_activation(self):
        self.activating_nods_timer.cancel()
        self.activating_nodes = False

    def configure_unconfigured_nodes(self):

        if all([ node['state'] == 'active' for node in self.nodes.values() ]):
            self.get_logger().info('Finished activating all nodes')
            self.stop_nodes_activation()
            return

        for node_name, node in self.nodes.items():

            # if all dependencies are active
            if node['state']=='unconfigured' and all([ self.nodes[dep]['state'] == 'active' for dep in node['depends_on'] ]):
                self.get_logger().info(f'{node_name} is unconfigured and all its dependent nodes are active. Starting...')
                self.trigger_node_transition(node_name, 'configure')

    def trigger_node_transition(self, node_name, transition):

        self.get_logger().info(f'Triggering {transition} for node {node_name}')
    
        def node_transition_callback(future):
            response = future.result()
            if response is None:
                self.get_logger().error(f'Failed to {transition} node {node_name}, response is None')
                return
            
            if not response.success:
                self.get_logger().error(f'Failed to {transition} node {node_name}')
                self.get_logger().info(f'response: {response}')
                return

            self.get_logger().info(f'{transition} request for {node_name} succeeded')
            # self.activate_node(node_name)
            if transition == 'configure':
                self.trigger_node_transition(node_name, 'activate')

        self.call_change_state(node_name, transition, lambda f: node_transition_callback(f))

    def call_change_state(self, node_name, transition, callback):

        client = self.create_client(ChangeState, f'/{node_name}/change_state')
        if not client.service_is_ready():
            self.get_logger().error(f'Could not change state of node {node_name}. Service not available.')
            return
        
        transition_label_to_id = {
            'configure': Transition.TRANSITION_CONFIGURE,
            'cleanup': Transition.TRANSITION_CLEANUP,
            'activate': Transition.TRANSITION_ACTIVATE,
            'deactivate': Transition.TRANSITION_DEACTIVATE,
        }
        
        # Create a request
        request = ChangeState.Request()
        request.transition.id = transition_label_to_id[transition]
        request.transition.label = transition
        
        future = client.call_async(request)
        future.add_done_callback(callback)

    def transition_event_callback(self, node_name, msg):
        state = msg.goal_state.label
        self.get_logger().info(f'get transition: {node_name} -> {state}')
        self.nodes[node_name]['state'] = state

nodes = [
    {
        'node_name': 'data_server',
        'depends_on': []
    },
    {
        'node_name': 'problem_generator',
        'depends_on': ['data_server']
    },
]

def main(args=None):
    rclpy.init(args=args)
    manager_node = LifecycleMannager()

    for node in nodes:
        manager_node.add_node(node['node_name'], node['depends_on'])
    manager_node.wait_for_nodes_and_start()

    rclpy.spin(manager_node)
    manager_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()