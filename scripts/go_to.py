#!/usr/bin/python3

# Copyright 2023 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.parameter import Parameter, ParameterType

from plansys2_support_py.ActionExecutorClient import ActionExecutorClient


class move(ActionExecutorClient):

    def __init__(self):
        super().__init__('move', 0.2)
        self.is_new_action = True

    # DO NOT USE THIS ↓↓↓ FUNCTIONS TO IMPLEMENT THE ACTION 
    def finish(self, success, completion, status):
        super().finish(success, completion, status)
        self.handle_end_of_action(success, completion, status)
        self.is_new_action = True

    def do_work(self):
        if self.is_new_action:
            self.is_new_action = False
            self.handle_start_of_action()
        self.handle_action_loop()
    # DO NOT USE THIS ↑↑↑ FUNCTIONS TO IMPLEMENT THE ACTION 
    
    def handle_start_of_action(self):
        self.get_logger().info('Starting action')
        self.progress_ = 0.0

    def handle_action_loop(self):

        self.get_logger().info(f'moveing ... {self.progress_:.2f}')

        self.progress_ += 0.1
        
        if self.progress_ < 1.0:
            self.send_feedback(self.progress_, 'move running')
        else:
            self.finish(True, 1.0, 'move completed')

    def handle_end_of_action(self, success, completion, status):
        if success:
            self.get_logger().info("Action finished successfully.")
        else:
            self.get_logger().info(f"Action finished with error: {status}")

def main(args=None):
    rclpy.init(args=args)

    node = move()
    node.set_parameters([Parameter(name='action_name', value='move')])

    node.trigger_configure()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
