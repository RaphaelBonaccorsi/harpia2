#!/usr/bin/env python3

import rclpy
import math
import json
from rclpy.node import Node
from harpia_msgs.srv import GeneratePath
from geometry_msgs.msg import PoseStamped
from harpia_msgs.action import MoveTo
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory


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
        with open(map_file, 'r') as file:
            data = json.load(file)

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


class RRT:
    """Rapidly-exploring Random Tree (RRT) for path planning."""

    def __init__(self, start, goal, map_limits, step_size=1.0, max_iters=5000):
        """
        Initializes the RRT with start and goal points and map constraints.

        Parameters
        ----------
        start : tuple
            Starting point for the path (x, y).
        goal : tuple
            Goal point for the path (x, y).
        map_limits : list
            Boundaries of the map [(x_min, y_min), (x_max, y_max)].
        step_size : float, optional
            Step size for RRT expansion (default is 1.0).
        max_iters : int, optional
            Maximum iterations for RRT expansion (default is 5000).
        """
        self.start = start
        self.goal = goal
        self.map_limits = map_limits
        self.step_size = step_size
        self.max_iters = max_iters

    def build(self):
        """
        Constructs a direct path between start and goal.

        Returns
        -------
        list
            List of points forming the path between start and goal.
        """
        path = self.generate_straight_path(self.start, self.goal)
        return path

    def generate_straight_path(self, start, goal):
        """
        Generates a straight path between start and goal with interpolated points.

        Parameters
        ----------
        start : tuple
            Starting point (x, y).
        goal : tuple
            Goal point (x, y).
        num_points : int, optional
            Number of interpolated points (default is 10).

        Returns
        -------
        list
            List of points along the straight path.
        """
        path = []
        path.append((start[0], start[1]))
        path.append((goal[0], goal[1]))
        return path


class PathPlanner(Node):
    """Path planning node for generating and executing paths."""

    def __init__(self):
        """Initializes the path planner node, map, and service."""
        super().__init__('path_planner')

        self.home_lat = -22.001333
        self.home_lon = -47.934152

        self.map = Map(self.home_lat, self.home_lon)
        try:
            package_share_dir = get_package_share_directory('route_executor2')
            self.map.read_route_from_json(f"{package_share_dir}/data/map.json")
            # self.map.read_route_from_json("/home/artur/rafael/route_executor2/data/map.json")
        except:
            self.get_logger().error("map.json not found, check the path and permissions.")

        self.srv = self.create_service(GeneratePath, 'path_planner/generate_path', self.generate_path_callback)
        self.get_logger().info("Path planner service is ready to generate paths.")

        ## self.action_client = ActionClient(self, MoveTo, '/drone/move_to_waypoint')
        

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
        map_limits = [(-10000, -10000), (10000, 10000)]
        rrt = RRT(start, goal, map_limits)
        path = rrt.build()

        # Verificar se o caminho foi gerado
        if not path:
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