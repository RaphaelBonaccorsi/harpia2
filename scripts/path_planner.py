#!/usr/bin/env python3

import rclpy
import math
import json
from rclpy.node import Node
from harpia_msgs.srv import GeneratePath, GetMap
from geometry_msgs.msg import PoseStamped
from harpia_msgs.action import MoveTo
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory
import time, os, sys

# Adiciona o diretório base ao sys.path
libs_path = os.path.join(os.path.dirname(__file__), './libs')
sys.path.append(os.path.abspath(libs_path))

from libs.RRT.rrt import RRT
from libs.RayCasting.raycasting import Vector
class CoordinateConverter:
    """Coordinate conversion utilities."""

    @staticmethod
    def geo_to_cart(geo_point, geo_home):
        """
        Converts geographic coordinates to Cartesian coordinates.

        Parameters
        ----------
        geo_point : tuple
            A tuple representing the latitude and longitude of the point to convert.
        geo_home : tuple
            A tuple representing the latitude and longitude of the home reference point.

        Returns
        -------
        tuple
            Cartesian coordinates (x, y) relative to the home reference point.
        """
        def calc_y(lat, home_lat):
            return (lat - home_lat) * 111320.0

        def calc_x(longi, home_long, home_lat):
            return (longi - home_long) * (111320.0 * math.cos(home_lat * math.pi / 180))

        x = calc_x(geo_point[1], geo_home[1], geo_home[0])  # Longitude
        y = calc_y(geo_point[0], geo_home[0])  # Latitude

        return x, y


class Map:
    """Map data handling with conversion and parsing capabilities."""

    def __init__(self, home_lat, home_lon):
        """
        Initializes the map with the given home coordinates.

        Parameters
        ----------
        home_lat : float
            Latitude of the home position.
        home_lon : float
            Longitude of the home position.
        """
        self.home_lat = home_lat
        self.home_lon = home_lon
        self.converter = CoordinateConverter()
        self.bases = []
        self.rois = []
        self.nfz = []

    def read_route_from_json(self, map_file):
        """
        Reads route data from a JSON file and populates bases, ROIs, and NFZs.

        Parameters
        ----------
        map_file : str
            Path to the JSON file containing route data.
        """
        
        data = json.loads(map_file)

        for base in data.get('bases', []):
            center = base.get('center')
            geo_points = base.get('geo_points', [])
            if center and geo_points:
                enu_center_x, enu_center_y = self.converter.geo_to_cart((center[1], center[0]), (self.home_lat, self.home_lon))
                enu_geo_points = [
                    self.converter.geo_to_cart((gp[1], gp[0]), (self.home_lat, self.home_lon)) for gp in geo_points
                ]
                self.bases.append({
                    'id': base.get('id'),
                    'name': base.get('name'),
                    'center': (enu_center_x, enu_center_y),
                    'geo_points': enu_geo_points
                })

        for roi in data.get('roi', []):
            center = roi.get('center')
            geo_points = roi.get('geo_points', [])
            if center and geo_points:
                enu_center_x, enu_center_y = self.converter.geo_to_cart((center[1], center[0]), (self.home_lat, self.home_lon))
                enu_geo_points = [
                    self.converter.geo_to_cart((gp[1], gp[0]), (self.home_lat, self.home_lon)) for gp in geo_points
                ]
                self.rois.append({
                    'id': roi.get('id'),
                    'name': roi.get('name'),
                    'center': (enu_center_x, enu_center_y),
                    'geo_points': enu_geo_points
                })

        for nfz in data.get('nfz', []):
            center = nfz.get('center')
            geo_points = nfz.get('geo_points', [])
            if center and geo_points:
                enu_center_x, enu_center_y = self.converter.geo_to_cart((center[1], center[0]), (self.home_lat, self.home_lon))
                enu_geo_points = [
                    self.converter.geo_to_cart((gp[1], gp[0]), (self.home_lat, self.home_lon)) for gp in geo_points
                ]
                self.nfz.append({
                    'id': nfz.get('id'),
                    'name': nfz.get('name'),
                    'center': (enu_center_x, enu_center_y),
                    'geo_points': enu_geo_points
                })

class PathPlanner(Node):
    """Path planning node for generating and executing paths."""

    def __init__(self):
        """Initializes the path planner node, map, and service."""
        super().__init__('path_planner')

        self.home_lat = -22.001333
        self.home_lon = -47.934152
        self.map_cli = self.create_client(GetMap, 'data_server/map')
        while not self.map_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('data_server/map service not available, waiting again...')
        
        self.req = GetMap.Request()
        self.req.request = True
        self.future = self.map_cli.call_async(self.req)
        self.future.add_done_callback(self.map_response_callback)
        
        self.map = Map(self.home_lat, self.home_lon)

        self.srv = self.create_service(GeneratePath, 'path_planner/generate_path', self.generate_path_callback)
        self.get_logger().info("Path planner service is ready to generate paths.")

        ## self.action_client = ActionClient(self, MoveTo, '/drone/move_to_waypoint')
        
    def map_response_callback(self, future):
        # Callback for the data_server/map service
        try:
            response = future.result()
            self.get_logger().info('Map response from data_server')
            self.map.read_route_from_json(response.map_file)
        except Exception as e:
            self.get_logger().error(f'Map data_server service call failed: {e}')
            self.finish(False, 0.0, 'Service call exception')

    def find_location_by_name(self, name):
        """
        Searches for a location by name in bases, ROIs, or NFZs.

        Parameters
        ----------
        name : str
            Name of the location to find.

        Returns
        -------
        tuple or None
            Coordinates (x, y) of the location, or None if not found.
        """
        for base in self.map.bases:
            if base['name'] == name:
                return base['center']

        for roi in self.map.rois:
            if roi['name'] == name:
                return roi['center']

        for nfz in self.map.nfz:
            if nfz['name'] == name:
                return nfz['center']

        return None

    def generate_path_callback(self, request, response):
        """
        Handles path generation requests and sets up waypoints.

        Parameters
        ----------
        request : GeneratePath.Request
            Service request containing origin and destination names.
        response : GeneratePath.Response
            Service response to be returned after path generation.

        Modifies
        --------
        response : GeneratePath.Response
            Populates the response object with waypoints.
        """
        self.get_logger().info(f"Received request to generate path from {request.origin} to {request.destination}")

        # Encontrar as coordenadas de início e destino
        start = self.find_location_by_name(request.origin)
        goal = self.find_location_by_name(request.destination)

        # Validar se as localizações existem
        if start is None or goal is None:
            self.get_logger().error("Invalid origin or destination name.")
            response.waypoints = []  # Resposta vazia em caso de erro
            return

        # Gerar o caminho usando RRT
        
        # Define the rand_area, based on origin and destination coordinates
        ps = [start[0], start[1], goal[0], goal[1]]
        rand_area_x = math.floor(min(ps) * 1.2)
        rand_area_y = math.ceil(max(ps) * 1.2)
        rand_area = [rand_area_x, rand_area_y]
        obstacleList = list()
        # Iterando pelos NFZs no mapa
        for nfz in self.map.nfz:
            # Lista temporária para armazenar os vetores de um único geo_point
            obstacle = []

            for geo_point in nfz.get('geo_points', []):  # Certifica-se de que 'geo_points' existe
                vector = Vector(geo_point[0], geo_point[1])  # Cria um Vector com x e y
                obstacle.append(vector)  # Adiciona o Vector à lista temporária

            obstacleList.append(obstacle)  # Adiciona o obstáculo completo à lista principal
    
        rrt = RRT(
        start=start,
        goal=goal,
        rand_area=rand_area,
        obstacle_list=obstacleList,
        expand_dis=25,  # minumum precision, to consider inside the goal (meters) 100
        path_resolution=1,
        goal_sample_rate=50,
        max_iter=5000,
        check_collision_mode="ray_casting",
        )

        path = rrt.planning(animation=False)
        if path is not None:
            path = list(reversed(path))
        else:
            self.get_logger().error("Failed to generate a valid path.")
            response.waypoints = []  # Resposta vazia se o caminho falhar
            return

        # Criar mensagens de waypoints para o caminho gerado
        waypoints = [self.create_waypoint_message(waypoint) for waypoint in path]
        response.waypoints = waypoints

        self.get_logger().info("Path generation complete.")
        return response



    def create_waypoint_message(self, waypoint):
        """
        Creates a PoseStamped message for a given waypoint.

        Parameters
        ----------
        waypoint : tuple
            (x, y) coordinates of the waypoint.

        Returns
        -------
        PoseStamped
            Message representing the waypoint.
        """
        wp_msg = PoseStamped()
        wp_msg.pose.position.x = waypoint[0]
        wp_msg.pose.position.y = waypoint[1]
        wp_msg.pose.position.z = 10.0
        wp_msg.pose.orientation.w = 1.0  # Garantindo uma orientação válida
        return wp_msg

def main(args=None):
    """
    Main function to initialize the path planner node and handle spinning.
    """
    rclpy.init(args=args)
    path_planner = PathPlanner()
    rclpy.spin(path_planner)
    path_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()