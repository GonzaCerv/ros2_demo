#!/usr/bin/env python3
# Copyright 2019 Open Source Robotics Foundation, Inc.
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

import os
import sys

import PyQt5
import rclpy
import rospkg
import signal

from ament_index_python.packages import get_package_share_directory
from ros_demo_interfaces.action import MoveRobot
from PyQt5.QtWidgets import *
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node

from PyQt5 import QtWidgets, uic


class GoalControllerClientActionClient(Node, QtWidgets.QMainWindow):

    def __init__(self):
        super().__init__('goal_controller_action_client')
        QtWidgets.QMainWindow.__init__(self)

        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(get_package_share_directory(
            'goal_controller_client'), 'demo.ui')
        # Extend the widget with all attributes and children from UI file
        uic.loadUi(ui_file, self)
        # Set object name.
        self.setObjectName('goal_controller_client')
        # Set callbacks.
        start_button = self.start_button
        start_button.clicked.connect(self.start_button_qt_callback)
        stop_button = self.stop_button
        stop_button.clicked.connect(self.stop_button_qt_callback)
        self.show()
        self._send_goal_future = None
        self._action_client = ActionClient(self, MoveRobot, 'goal_controller')

    # Callbacks for start button.
    def start_button_qt_callback(self):
        self.get_logger().info(
            f'Sending goal to (x:{self.x_box.value()}, y:{self.y_box.value()} ,theta:{self.theta_box.value()}')
        self.send_goal(self.x_box.value(), self.y_box.value(), self.theta_box.value())

    # Callbacks for start button.
    def stop_button_qt_callback(self):
        if self._send_goal_future.done():
            self.get_logger().info(
                'Stopping robot')
            self._action_client
            self._send_goal_future.result().cancel_goal()
            self.get_logger().info(
                'No goal is running on server')
        else:
            self.get_logger().info(
                'Future did not end')

    def send_goal(self, x:float, y:float, theta:float):
        goal_msg = MoveRobot.Goal()
        goal_msg.target_pos.x = x
        goal_msg.target_pos.y = y
        goal_msg.target_pos.theta = theta

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Goal reached'.format(result.sequence))
        else:
            self.get_logger().info('Goal not reached'.format(result.sequence))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            'Distance to goal: {0}'.format(feedback.distance_to_goal))


def main(args=None):
    rclpy.init(args=args)

    # Allow node to be killed by Ctrl-C.
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    # Execute Application.
    app = QtWidgets.QApplication(sys.argv)
    action_client = GoalControllerClientActionClient()
    app.exec_()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
