#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from plansys2_msgs.msg import Plan

class PlanSubscriber(Node):

    def __init__(self):
        super().__init__('plan_subscriber')
        self.subscription = self.create_subscription(
            Plan,
            'plansys2_interface/plan',
            self.plan_callback,
            10)
        self.subscription  # evita warning de variável não utilizada

    def plan_callback(self, msg):
        self.get_logger().info('Plano recebido:')
        for action in msg.items:
            self.get_logger().info(f'Ação: {action.action}, Tempo inicial: {action.time:.2f}')

def main(args=None):
    rclpy.init(args=args)
    
    # Inicializa o nó
    plan_subscriber = PlanSubscriber()

    # Aguarda mensagens e processa callbacks
    rclpy.spin(plan_subscriber)

    # Encerra o nó
    plan_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
