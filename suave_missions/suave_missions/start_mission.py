#!/usr/bin/env python
import sys
import rclpy
import threading
import os
from datetime import datetime
from rclpy.action import ActionClient
from mros2_msgs.action import ControlQos
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from system_modes_msgs.srv import ChangeMode

from suave_missions.mission_planner import MissionPlanner


class Mission(MissionPlanner):
    def __init__(self, node_name='mission_node'):
        super().__init__(node_name)
        self.get_logger().info('INSPECTION MISSION')

    def perform_mission(self):
        self.get_logger().info("Pipeline inspection mission starting!!")

        self.timer = self.create_rate(1)

        while not self.status.armed:
            self.get_logger().info(
                'BlueROV is armed: {}'.format(self.status.armed))
            self.arm_motors(True)
            self.timer.sleep()

        guided_mode = 'GUIDED'
        while self.status.mode != guided_mode:
            self.get_logger().info(
                'BlueROV mode is : {}'.format(self.status.mode))
            self.set_mode(guided_mode)
            self.timer.sleep()



def main():

    rclpy.init(args=sys.argv)

    mission_node = Mission()

    mt_executor = MultiThreadedExecutor()
    thread = threading.Thread(
        target=rclpy.spin, args=[mission_node, mt_executor], daemon=True)
    thread.start()

    mission_node.perform_mission()

    thread.join()
    mission_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
