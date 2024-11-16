#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from math import sqrt
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Float32
from harpia_msgs.action import MoveTo
import time

class RouteExecutor(Node):
    def __init__(self):
        super().__init__('route_executor')

        # Publisher e clientes de serviço
        self.publisher_ = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Espera os serviços estarem disponíveis
        self.get_logger().info('Waiting for arming and set mode services...')
        self.arming_client.wait_for_service(timeout_sec=10.0)
        self.set_mode_client.wait_for_service(timeout_sec=10.0)

        # Inicializa variáveis de waypoint e posição atual
        self.waypoint = PoseStamped()
        self.current_position = PoseStamped()

        # Define QoS profile with reliable reliability to match MAVROS
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Configura o assinante da posição atual
        self.position_subscriber = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.position_callback, qos_profile
        )

        # Configura o servidor de ação
        self._action_server = ActionServer(
            self,
            MoveTo,
            '/drone/move_to_waypoint',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        

        # Configuração do timer de setpoint
        self.setpoint_timer = self.create_timer(0.05, self.publish_current_setpoint)
        self.get_logger().info('Started publishing setpoints at 20Hz...')

        # Ativa o modo Offboard e arma o drone
        time.sleep(2)
        self.set_offboard_mode()
        self.arm()

    def goal_callback(self, goal_request):
        self.get_logger().info(f'Recebendo pedido de move para: {goal_request.destination.pose}')
        self.waypoint = goal_request.destination
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancelando meta')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executando ação de movimentação')

        # Inicializa a mensagem de feedback
        feedback_msg = MoveTo.Feedback()
        
        # Loop de execução até o drone alcançar o waypoint ou a ação ser cancelada
        while not self.has_reached_waypoint(self.waypoint):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Ação cancelada')
                return MoveTo.Result(success=False)

            # Atualiza a distância no feedback e publica
            feedback_msg.distance = float(self.get_distance(self.waypoint))  # Verifica se feedback_msg.distance existe
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Distância até o waypoint: {feedback_msg.distance:.2f}m')
            time.sleep(1)  # Intervalo para simular o tempo de movimento

        # Se o objetivo foi alcançado
        goal_handle.succeed()
        result = MoveTo.Result()
        result.success = True
        self.get_logger().info('Movimentação concluída com sucesso')
        return result



    def position_callback(self, msg):
        """Callback para atualizar a posição atual do drone."""
        self.current_position = msg

    def get_current_position(self):
        """Retorna a posição atual do drone."""
        return self.current_position.pose.position

    def publish_current_setpoint(self):
        """Publica o próximo setpoint."""
        msg = PoseStamped()
        waypoint = self.waypoint

        msg.pose.position.x = waypoint.pose.position.x
        msg.pose.position.y = waypoint.pose.position.y
        msg.pose.position.z = waypoint.pose.position.z
        msg.pose.orientation.w = 1.0

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published setpoint: ({msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z})")

    def set_offboard_mode(self):
        """Define o modo do drone para OFFBOARD."""
        self.get_logger().info('Setting mode to OFFBOARD')
        set_mode_request = SetMode.Request()
        set_mode_request.custom_mode = 'OFFBOARD'
        set_mode_future = self.set_mode_client.call_async(set_mode_request)
        rclpy.spin_until_future_complete(self, set_mode_future)

        if set_mode_future.result() is not None and set_mode_future.result().mode_sent:
            self.get_logger().info('Offboard mode set successfully')
        else:
            self.get_logger().error('Failed to set Offboard mode')

    def arm(self):
        """Arma o drone."""
        self.get_logger().info('Arming the drone...')
        arm_request = CommandBool.Request()
        arm_request.value = True
        arm_future = self.arming_client.call_async(arm_request)
        rclpy.spin_until_future_complete(self, arm_future)

        if arm_future.result() is not None and arm_future.result().success:
            self.get_logger().info('Drone armed successfully')
        else:
            self.get_logger().error('Failed to arm the drone')

    def has_reached_waypoint(self, waypoint, threshold=1.0):
        """Verifica se o drone chegou ao waypoint."""
        distance = self.get_distance(waypoint)
        return distance < threshold

    def get_distance(self, waypoint):
        """Calcula a distância da posição atual até o waypoint."""
        current_position = self.get_current_position()
        distance = sqrt(
            (waypoint.pose.position.x - current_position.x) ** 2 +
            (waypoint.pose.position.y - current_position.y) ** 2 +
            (waypoint.pose.position.z - current_position.z) ** 2
        )
        return distance


def main(args=None):
    rclpy.init(args=args)
    node = RouteExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
