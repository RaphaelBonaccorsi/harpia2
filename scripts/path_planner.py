#!/usr/bin/env python3

import rclpy
import math
import json  # Certifique-se de importar o módulo json
from rclpy.node import Node
from harpia_msgs.srv import GeneratePath
from geometry_msgs.msg import PoseStamped


class CoordinateConverter:
    @staticmethod
    def geo_to_cart(geo_point, geo_home):
        def calc_y(lat, home_lat):
            return (lat - home_lat) * 111320.0

        def calc_x(longi, home_long, home_lat):
            return (longi - home_long) * (111320.0 * math.cos(home_lat * math.pi / 180))

        x = calc_x(geo_point[1], geo_home[1], geo_home[0])  # Longitude
        y = calc_y(geo_point[0], geo_home[0])  # Latitude

        return x, y


class Map:
    def __init__(self, home_lat, home_lon):
        self.home_lat = home_lat
        self.home_lon = home_lon
        self.converter = CoordinateConverter()
        self.bases = []
        self.rois = []
        self.nfz = []

    def read_route_from_json(self, map_file):
        with open(map_file, 'r') as file:
            data = json.load(file)

        # Processar as bases
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

        # Processar as ROIs (Regiões de Interesse)
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

        # Processar as NFZs (No Fly Zones)
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
    def __init__(self, start, goal, map_limits, step_size=1.0, max_iters=5000):
        self.start = start
        self.goal = goal
        self.map_limits = map_limits  # Limites do mapa [(x_min, y_min), (x_max, y_max)]
        self.step_size = step_size
        self.max_iters = max_iters

    def build(self):
        """Constrói uma rota direta entre start e goal."""
        path = self.generate_straight_path(self.start, self.goal)
        return path

    def generate_straight_path(self, start, goal, num_points=10):
        """Gera uma rota reta entre start e goal com num_points pontos de interpolação."""
        path = []
        for i in range(num_points + 1):
            t = i / num_points
            x = (1 - t) * start[0] + t * goal[0]
            y = (1 - t) * start[1] + t * goal[1]
            path.append((x, y))
        return path


class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        self.home_lat = -22.001333
        self.home_lon = -47.934152

        # Initialize Map object for handling waypoints
        self.map = Map(self.home_lat, self.home_lon)

        # Exemplo: carregar um mapa fictício
        self.map.read_route_from_json("/home/harpia/route_executor2/data/map.json")

        # Create the service
        self.srv = self.create_service(GeneratePath, 'path_planner/generate_path', self.generate_path_callback)
        self.get_logger().info("Path planner service is ready to generate paths.")

        # Create a publisher to send waypoints to 'drone/waypoints'
        self.waypoint_pub = self.create_publisher(PoseStamped, 'drone/waypoint', 10)

    def find_location_by_id(self, id):
        """Busca um local (base ou ROI) pelo ID."""
        for base in self.map.bases:
            if base['id'] == id:
                return base['center']

        for roi in self.map.rois:
            if roi['id'] == id:
                return roi['center']

        return None

    def generate_path_callback(self, request, response):
        self.get_logger().info(f"Received request to generate path from {request.origin_id} to {request.destination_id}")

        # Buscar o local de origem e destino (pode ser uma base ou ROI)
        start = self.find_location_by_id(request.origin_id)
        goal = self.find_location_by_id(request.destination_id)

        if start is None or goal is None:
            self.get_logger().error("Invalid origin or destination ID.")
            return response

        # Definir limites do mapa (exemplo: [(x_min, y_min), (x_max, y_max)])
        map_limits = [(-1000, -1000), (1000, 1000)]  # Defina os limites apropriados do mapa

        # Inicializar o algoritmo RRT (atualmente apenas gera uma linha reta)
        rrt = RRT(start, goal, map_limits)

        # Gerar o caminho
        path = rrt.build()

        # Adicionar os waypoints ao response como PoseStamped e publicar cada um
        for waypoint in path:
            wp_msg = PoseStamped()
            wp_msg.pose.position.x = waypoint[0]
            wp_msg.pose.position.y = waypoint[1]
            wp_msg.pose.position.z = 10.0  # Assumindo uma altitude constante para simplificação
            response.waypoints.append(wp_msg)

            # Publica o waypoint no tópico 'drone/waypoints'
            self.waypoint_pub.publish(wp_msg)
            self.get_logger().info(f"Published waypoint: x={wp_msg.pose.position.x}, y={wp_msg.pose.position.y}, z={wp_msg.pose.position.z}")

        self.get_logger().info("Path generation and publishing complete.")
        return response  # Retorna sempre o objeto response


def main(args=None):
    rclpy.init(args=args)

    # Create the node and spin
    path_planner = PathPlanner()
    rclpy.spin(path_planner)

    # Shutdown the node after it's done
    path_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
