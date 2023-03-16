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


class RechargeLC(Node):

    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)
        self.trigger_configure()
        self.abort_follow = False
        self.distance_inspected = 0
        self.recharge_location = PoseStamped()
        self.recharge_location.pose.position.x = 1.0
        self.recharge_location.pose.position.y = 1.0
        self.recharge_location.pose.position.z = -0.5
        self.goal_setpoint_local = PoseStamped()
        self.goal_setpoint_local.pose.position.x = -100.0
        self.goal_setpoint_local.pose.position.y = -100.0
        self.goal_setpoint_local.pose.position.z = -100.0
        self.xyz = None
        self.count = 0
        self.i = 0
        self.ardusub = BlueROVGazebo('bluerov_recharge')
        self.thread = threading.Thread(
            target=rclpy.spin, args=(self.ardusub, ), daemon=True)
        self.thread.start()

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure() is called.")

        self.battery_cli = self.create_client(SetBool, 'battery_monitor')
        self.setpoint_pub = self.create_publisher(
            PoseStamped, 'maintain_motion/setpoint', 10)
        self.recharge_pub = self.create_publisher(
            Bool, 'recharge/recharge_done', 10)
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
            self.executor.create_task(self.recharge)
            self.abort_follow = False

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate() is called.")
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_cleanup() is called.')
        self.ardusub.destroy_node()
        self.thread.join()
        self.recharge_thread.join()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_shutdown() is called.')
        self.ardusub.destroy_node()
        self.thread.join()
        self.recharge_thread.join()
        return TransitionCallbackReturn.SUCCESS

    def recharge(self):
        self.get_logger().info("Recharge started")
        timer = self.ardusub.create_rate(1)  # Hz

        # timer = self.ardusub.create_rate(5)  # Hz

        self.distance_inspected = 0

        current_loc = self.ardusub.local_pos
        self.get_logger().info(str(current_loc))
        while (current_loc.pose.position.x+current_loc.pose.position.y+current_loc.pose.position.z) == 0.0:
            current_loc = self.ardusub.local_pos
            self.get_logger().info(str(current_loc))
            timer.sleep()

        self.get_logger().info('after while')
        dist = self.calc_distance(self.recharge_location, current_loc)
        
        idxs = round(dist/0.5)
        dx = self.recharge_location.pose.position.x - current_loc.pose.position.x
        dy = self.recharge_location.pose.position.y - current_loc.pose.position.y
        dz = self.recharge_location.pose.position.z - current_loc.pose.position.z
        pose = PoseStamped()
        pose = current_loc
        poses = [pose]
        self.get_logger().info('before for')
        for _ in range(1,idxs):
            pose.pose.position.x += dx
            pose.pose.position.y += dy
            pose.pose.position.z += dz
            poses.append(pose)
        
        self.get_logger().info(str(poses))
        self.ardusub.altitude = 3.0
        # while not self.ardusub.check_setpoint_reached_xy(self.recharge_location):
        while self.i < idxs:
            gz_pose = poses[self.i]
            if self.count > 10:
                self.count = 0
                self.setpoint_pub.publish(gz_pose)
                self.get_logger().info('published x='+
                    str(gz_pose.pose.position.x)+', y='+str(gz_pose.pose.position.y)+
                    ', z='+str(gz_pose.pose.position.z))
                self.count = 0

            if self.xyz == None or self.ardusub.check_setpoint_reached_xy(self.goal_setpoint_local, 0.4):
                self.xyz = self.ardusub.calc_position_local(gz_pose.pose.position.x,gz_pose.pose.position.y,fixed_altitude=True)
                self.goal_setpoint_local.pose.position.x = self.xyz[0]
                self.goal_setpoint_local.pose.position.y = self.xyz[1]
                self.goal_setpoint_local.pose.position.z = self.xyz[2]
                self.setpoint_pub.publish(gz_pose)
                self.get_logger().info('published x='+
                    str(gz_pose.pose.position.x)+', y='+str(gz_pose.pose.position.y)+
                    ', z='+str(gz_pose.pose.position.z))
                self.count = 0
                self.i += 1
            else:
                self.get_logger().info('added count')
                self.count += 1
            timer.sleep()

        self.get_logger().info("Recharge completed")
        msg = Bool()
        msg.data = True
        self.recharge_pub.publish(msg)

        req = SetBool.Request()
        req.data = False # recharge needed set to False
        try:
            battery_call_response = self.battery_cli.call(req)
        except Exception as e:
            self.logger().info('Request creation failed %r' % (e,))
            return None
        else:
            return battery_call_response


    def calc_distance(self, pose1, pose2):
        return math.sqrt(
            (pose1.pose.position.x - pose2.pose.position.x)**2 + (pose1.pose.position.y - pose2.pose.position.y)**2 +
            (pose1.pose.position.z - pose2.pose.position.z)**2)


def main():
    rclpy.init()

    executor = rclpy.executors.MultiThreadedExecutor()
    lc_node = RechargeLC('f_recharge_wp_node')
    executor.add_node(lc_node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        lc_node.destroy_node()


if __name__ == '__main__':
    main()
