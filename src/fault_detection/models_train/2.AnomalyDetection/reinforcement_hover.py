"""
testing offboard positon control with a simple takeoff script
Author: Lukas Vacek @ https://github.com/darknight-007/docking
"""

import rclpy
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, TwistStamped
import math
import numpy
from VelocityController import VelocityController
from LandingController import LandingController
import random
import time

from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_srvs.srv import SetBool


class QuadController(Node):
    cur_vel = TwistStamped()
    des_vel = TwistStamped()
    sim_ctr = 1

    des_pose = PoseStamped()
    cur_pose = PoseStamped()

    isReadyToFly = True

    target = Pose()
    target.position.x = 7
    target.position.y = 3
    target.position.z = 5
    landed = False

    def __init__(self):
        super().__init__('f450_velocity_controller')
        self.set_parameters([{"mavros/conn/heartbeat_rate": '3.0'}])
        qos_profile = QoSProfile(depth=10)
        vel_pub = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', qos_profile)
        vel_sub = self.create_subscription(TwistStamped, '/mavros/local_position/velocity', self.vel_cb, qos_profile)
        pos_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pos_cb, qos_profile)
        ext_state_sub = self.create_subscription(ExtendedState, '/mavros/extended_state', self.ext_state_cb, qos_profile)
        state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, qos_profile)

        arming_srv = self.create_client(CommandBool, "/mavros/cmd/arming")
        mode_srv = self.create_client(SetMode, "/mavros/set_mode")

        rate = self.create_rate(10)  # Hz
        
        lController = LandingController()
        lController.setTarget(self.target)
        self.des_vel = lController.update(self.cur_pose, self.cur_vel)
        # self.des_vel.twist.linear.x = 0
        # self.des_vel.twist.linear.y = 0
        for i in range(0,10):
            vel_pub.publish(self.des_vel)
            rate.sleep()

        print("Setting Offboard Mode")
        result = mode_srv.call_async(SetMode.Request(custom_mode="OFFBOARD"))
        print(result.result())
        print("Arming")
        result = arming_srv.call_async(CommandBool.Request(value=True))
        print(result.result())
        self.perturbation_flag = 0
        self.hover_point_flag = 0

        while rclpy.ok():

            if self.isReadyToFly:
                self.des_vel = lController.update(self.cur_pose, self.cur_vel)
                vel_pub.publish(self.des_vel)
                
                print("----------------------------------current pose after this-------------------------")
                print(self.cur_pose)
                print("----------------------------------target after this-------------------------------")
                print(self.target)
                
                if((self.target.position.x - 0.2 <=self.cur_pose.pose.position.x <= self.target.position.x + 0.2) and (self.target.position.y - 0.2 <=self.cur_pose.pose.position.y <= self.target.position.y + 0.2)):
                    print("reached")
                    if (self.cur_pose.pose.position.x <= 7 + 0.1 and self.cur_pose.pose.position.y <=3 + 0.1 and self.cur_pose.pose.position.z <= 8 + 0.1):
                        print("you are at hover position")
                        time.sleep(3)
                        self.hover_point_flag = 1
                    else:
                        print("you are at perturbed position")
                        print("Updating targets to go back to hover point")
                        self.target.position.x = 7
                        self.target.position.y = 3  
                        self.target.position.z = 5
                        self.hover_point_flag = 0

                if self.hover_point_flag == 1:    
                    print("adding perturbation")
                    self.target.position.x = 7+random.random()*10
                    self.target.position.y = 3+random.random()*10  
                    self.target.position.z = 5+random.random()*2 
                    self.hover_point_flag = 0  
                    # DISARM
                    #result = arming_srv.call_async(CommandBool.Request(value=False))
                    #break
            rate.sleep()

    def copy_vel(self, vel):
        copied_vel = TwistStamped()
        copied_vel.header= vel.header
        return copied_vel

    def vel_cb(self, msg):
        # print msg
        self.cur_vel = msg

    def pos_cb(self, msg):
        # print msg
        self.cur_pose = msg

    def state_cb(self,msg):
        if(msg.mode=='OFFBOARD'):
            self.isReadyToFly = True

    def ext_state_cb(self, msg):
        if(msg.landed_state == 1):
            self.landed = True

def main(args=None):
    rclpy.init(args=args)
    quad_controller = QuadController()
    rclpy.spin(quad_controller)

    quad_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
