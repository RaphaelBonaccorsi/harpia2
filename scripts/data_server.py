#!/usr/bin/env python3

import rclpy
import math
import json
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
import time, os, sys
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class DataServer(Node):

    def __init__(self):

        super().__init__('data_server')
        package_share_dir = get_package_share_directory('route_executor2')
        self.map_file = f"{package_share_dir}/data/map.json"
        self.hardware_file = f"{package_share_dir}/data/hardware.json"
        self.all_missions_file = f"{package_share_dir}/data/all_missions.json"
        self.mission_index = 1
        # Define QoS profile for the position subscription
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        # Subscribe to the drone's current position
        self.position_subscriber = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.position_callback, qos_profile
        )
        self.map_srv = self.create_service(Trigger, 'data_server/map', self.map_callback)
        self.mission_srv = self.create_service(Trigger, 'data_server/mission', self.mission_callback)
        self.hardware_srv = self.create_service(Trigger, 'data_server/hardware', self.hardware_callback)
        self.position_srv = self.create_service(Trigger, 'data_server/drone_position', self.position_service_callback)

        self.get_logger().info("Data server service is ready.")

        ## self.action_client = ActionClient(self, MoveTo, '/drone/move_to_waypoint')
    
    def position_service_callback(self, request, response):
        response.message = f'{self.current_position.x} {self.current_position.y} {self.current_position.z}'
        return response
    
    def position_callback(self, msg):
        """
        Updates the drone's current position.

        Parameters
        ----------
        msg : PoseStamped
            The current position of the drone.
        """
        self.current_position = msg.pose.position

    def map_callback(self, request, response):
        self.get_logger().info(f"Received request to send map")
        with open(self.map_file, 'r') as file:
            response.message = file.read()
        return response

    def mission_callback(self, request, response):
        self.get_logger().info(f"Received request to send mission")
        with open(self.all_missions_file, 'r') as file:
            missions = json.load(file)
            response.message = json.dumps(missions[self.mission_index])
        return response

    def hardware_callback(self, request, response):
        self.get_logger().info(f"Received request to send hardware")
        with open(self.hardware_file, 'r') as file:
            response.message = file.read()
        return response

def main(args=None):
    """
    Main function to initialize the data_server node and handle spinning.
    """
    rclpy.init(args=args)
    data_server = DataServer()
    rclpy.spin(data_server)
    data_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()