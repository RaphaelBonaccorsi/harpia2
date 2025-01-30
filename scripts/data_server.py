#!/usr/bin/env python3

import rclpy
import math
import json
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
import time, os, sys
from std_srvs.srv import Trigger

class DataServer(Node):

    def __init__(self):

        super().__init__('data_server')
        package_share_dir = get_package_share_directory('route_executor2')
        self.map_file = f"{package_share_dir}/data/map.json"
        self.hardware_file = f"{package_share_dir}/data/hardware.json"
        self.mission_file = f"{package_share_dir}/data/mission.json"

        self.map_srv = self.create_service(Trigger, 'data_server/map', self.map_callback)
        self.mission_srv = self.create_service(Trigger, 'data_server/mission', self.mission_callback)
        self.hardware_srv = self.create_service(Trigger, 'data_server/hardware', self.hardware_callback)
        self.get_logger().info("Data server service is ready.")

        ## self.action_client = ActionClient(self, MoveTo, '/drone/move_to_waypoint')
    
    def map_callback(self, request, response):
        self.get_logger().info(f"Received request to send map")
        with open(self.map_file, 'r') as file:
            response.message = file.read()
        return response

    def mission_callback(self, request, response):
        self.get_logger().info(f"Received request to send mission")
        with open(self.mission_file, 'r') as file:
            response.message = file.read()
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