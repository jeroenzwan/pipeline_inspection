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

from suave_msgs.srv import GetPath
from suave.bluerov_gazebo import BlueROVGazebo

from std_msgs.msg import Bool
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped


class PipelineFollowerLC(Node):

    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)
        self.trigger_configure()
        self.distance_inspected = 0
        self.goal_setpoint_local = PoseStamped()
        self.goal_setpoint_local.pose.position.x = -100.0
        self.goal_setpoint_local.pose.position.y = -100.0
        self.goal_setpoint_local.pose.position.z = -100.0
        self.count = 0
        self.i = 0
        self.xyz = None
        self.height = 3.0

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure() is called.")
        self.ardusub = BlueROVGazebo('bluerov_pipeline_follower')
        self.thread = threading.Thread(
            target=rclpy.spin, args=(self.ardusub, ), daemon=True)
        self.thread.start()

        self.get_path_timer = self.create_rate(5)
        self.get_path_service = self.create_client(
            GetPath, 'pipeline/get_path')

        self.pipeline_inspected_pub = self.create_lifecycle_publisher(
            Bool, 'pipeline/inspected', 10)

        self.pipeline_distance_inspected_pub = self.create_publisher(
            Float32, 'pipeline/distance_inspected', 10)

        self.setpoint_pub = self.create_publisher(
            PoseStamped, 'maintain_motion/setpoint', 10)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate() is called.")
        if not self.get_path_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                'pipeline/get_path service is not available')
            return TransitionCallbackReturn.FAILURE
        if self.executor is None:
            self.get_logger().info('Executor is None')
            return TransitionCallbackReturn.FAILURE
        else:
            self.executor.create_task(self.follow_pipeline)

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate() is called.")
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_cleanup() is called.')
        self.ardusub.destroy_node()
        self.thread.join()
        self.follow_pipeline_thread.join()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_shutdown() is called.')
        self.ardusub.destroy_node()
        self.thread.join()
        self.follow_pipeline_thread.join()
        return TransitionCallbackReturn.SUCCESS

    def follow_pipeline(self):
        self.get_logger().info("Follow pipeline started")
        timer = self.ardusub.create_rate(1)  # Hz

        cur_pose = self.ardusub.local_pos.pose
        self.get_logger().info(str(cur_pose))
        while (cur_pose.position.x+cur_pose.position.y+cur_pose.position.z) == 0.0:
            cur_pose = self.ardusub.local_pos.pose
            self.get_logger().info(str(cur_pose))
            timer.sleep()
        self.get_logger().info("After while loop")

        req = GetPath.Request()
        pipe_path = self.get_path_service.call_async(req)

        while not pipe_path.done():
            pipe_path = self.get_path_service.call_async(req)
            timer.sleep()

        self.distance_inspected = 0
        self.distance_to_inspect = 5
        gz_poses = pipe_path.result().path.poses
        self.get_logger().info('First pose is x='+
            str(gz_poses[0].position.x)+', y='+str(gz_poses[0].position.y)+
            ', z='+str(gz_poses[0].position.z))

        last_point = None
        self.ardusub.altitude = self.height
        while self.distance_inspected < self.distance_to_inspect:
            self.get_logger().info('Distance covered '+str(self.distance_inspected))
            pose = PoseStamped()
            pose.pose = gz_poses[self.i]

            if self.count > 10:
                self.count = 0
                self.setpoint_pub.publish(pose)
                self.get_logger().info('published x='+
                    str(pose.pose.position.x)+', y='+str(pose.pose.position.y)+
                    ', z='+str(pose.pose.position.z))
                self.count = 0

            if self.xyz == None or self.ardusub.check_setpoint_reached_xy(self.goal_setpoint_local, 0.4):
                self.xyz = self.ardusub.calc_position_local(pose.pose.position.x,pose.pose.position.y,fixed_altitude=True)
                self.goal_setpoint_local.pose.position.x = self.xyz[0]
                self.goal_setpoint_local.pose.position.y = self.xyz[1]
                self.goal_setpoint_local.pose.position.z = self.xyz[2]
                pose.pose.position.z = self.height
                self.setpoint_pub.publish(pose)
                self.get_logger().info('published x='+
                    str(pose.pose.position.x)+', y='+str(pose.pose.position.y)+
                    ', z='+str(pose.pose.position.z))
                self.count = 0
                self.i += 1
                if last_point is not None:
                    self.distance_inspected += self.calc_distance(last_point, pose)
                last_point = pose
            else:
                self.get_logger().info('added count')
                self.count += 1
            timer.sleep()

        pipe_inspected = Bool()
        pipe_inspected.data = True
        self.pipeline_inspected_pub.publish(pipe_inspected)
        self.get_logger().info("Follow pipeline completed")

    def calc_distance(self, pose1, pose2):
        return math.sqrt(
            (pose1.pose.position.x - pose2.pose.position.x)**2 +
            (pose1.pose.position.y - pose2.pose.position.y)**2)


def main():
    rclpy.init()

    executor = rclpy.executors.MultiThreadedExecutor()
    lc_node = PipelineFollowerLC('f_follow_pipeline_wp_node')
    executor.add_node(lc_node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        lc_node.destroy_node()


if __name__ == '__main__':
    main()
