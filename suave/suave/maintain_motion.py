#!/usr/bin/env python
from typing import Optional

import math
import rclpy
import threading

from rclpy.lifecycle import Node
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer

from suave.bluerov_gazebo import BlueROVGazebo

from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped


class MaintainMotionLC(Node):

    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)
        self.trigger_configure()
        self.abort_follow = False
        self.distance_inspected = 0
        self.ardusub = BlueROVGazebo('bluerov_maintain_motion')
        self.thread = threading.Thread(
            target=rclpy.spin, args=(self.ardusub, ), daemon=True)
        self.thread.start()

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure() is called.")

        self.battery_cli = self.create_client(SetBool, 'battery_monitor')
        self.setpoint_sub = self.create_subscription(
            PoseStamped, 'maintain_motion/setpoint', self.setpoint_cb, 10)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate() is called.")
        if not self.battery_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
            return TransitionCallbackReturn.FAILURE
        if self.executor is None:
            self.get_logger().info('Executor is None')
            return TransitionCallbackReturn.FAILURE
        else:
            self.executor.create_task(self.maintain_motion)
            self.abort_follow = False

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate() is called.")
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_cleanup() is called.')
        self.ardusub.destroy_node()
        self.thread.join()
        self.maintain_motion_thread.join()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_shutdown() is called.')
        self.ardusub.destroy_node()
        self.thread.join()
        self.maintain_motion_thread.join()
        return TransitionCallbackReturn.SUCCESS
    
    def setpoint_cb(self, msg):
        self.ardusub.altitude = msg.pose.position.z
        self.ardusub.setpoint_position_local(x=msg.pose.position.x,y=msg.pose.position.y, fixed_altitude=True)


    def maintain_motion(self):
        self.get_logger().info("Maintain Motion started")


    def calc_distance(self, pose1, pose2):
        return math.sqrt(
            (pose1[0] - pose2[0])**2 + (pose1[1] - pose2[1])**2 +
            (pose1[2] - pose2[2])**2)


def main():
    rclpy.init()

    executor = rclpy.executors.MultiThreadedExecutor()
    lc_node = MaintainMotionLC('f_maintain_motion_node')
    executor.add_node(lc_node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        lc_node.destroy_node()


if __name__ == '__main__':
    main()
