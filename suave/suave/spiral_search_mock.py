from typing import Optional

import rclpy

from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.lifecycle import Node
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer
from suave.bluerov_gazebo import BlueROVGazebo

import std_msgs.msg
import threading
import math


def spiral_points(i, old_x, old_y, resolution=0.1, spiral_width=1.0):
    if i == 0:
        return .0, .0
    else:
        delta_angle = i*resolution - (i-1)*resolution
        old_radius = math.sqrt(old_x**2 + old_y**2)
        current_radius = old_radius + (spiral_width*delta_angle/(2*math.pi))
        x = current_radius*math.cos(i*resolution)
        y = current_radius*math.sin(i*resolution)
        return x, y


class SpiralSearcherLC(Node):

    def __init__(self, node_name, **kwargs):
        self._enabled = False
        self.spiral_count: int = 0
        self.spiral_x: float = 0.0
        self.spiral_y: float = 0.0
        self.timer_period = 1.0
        self._timer: Optional[Timer] = None
        self.count = 0
        self.z_delta = 0
        self.old_spiral_altitude = -1
        self.abort_search = False

        super().__init__(node_name, **kwargs)

        self.goal_setpoint = None

        spiral_altitude_descriptor = ParameterDescriptor(
            description='Sets the spiral altitude of the UUV.')
        self.declare_parameter(
            'spiral_altitude', 2.0, spiral_altitude_descriptor)

        self.param_change_callback_handle = \
            self.add_on_set_parameters_callback(self.param_change_callback)

        self.trigger_configure()

    def param_change_callback(self, parameters):
        result = SetParametersResult()
        result.successful = True
        for parameter in parameters:
            self.get_logger().info(
                "parameter '{}' is now: {}".format(
                    parameter.name,
                    parameter.value))
        return result


    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_configure() is called.')
        self.ardusub = BlueROVGazebo('bluerov_spiral_search')

        self.thread = threading.Thread(
            target=rclpy.spin, args=(self.ardusub, ), daemon=True)
        self.thread.start()

        self.detect_pipeline_pub = self.create_publisher(
            Bool, 'pipeline/detected', 10)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate() is called.")
        self._enabled = True

        if self.executor is None:
            self.get_logger().info('Executor is None')
            return TransitionCallbackReturn.FAILURE
        else:
            self.executor.create_task(self.search_pipeline)
            self.abort_search = False

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate() is called.")
        self._enabled = False
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.thread.join()
        self.ardusub.destroy_node()
        self.destroy_timer(self._timer)
        self.get_logger().info('on_cleanup() is called.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.thread.join()
        self.ardusub.destroy_node()
        self.destroy_timer(self._timer)
        self.get_logger().info('on_shutdown() is called.')
        return TransitionCallbackReturn.SUCCESS

    def search_pipeline(self):
        self.get_logger().info("Search pipeline started")
        timer = self.ardusub.create_rate(0.2)  # Hz
        timer.sleep()

        pipe_found = Bool()
        pipe_found.data = True
        self.pipeline_found.publish(pipe_found)
        self.get_logger().info("Search pipeline completed")

    def calc_distance(self, pose1, pose2):
        return math.sqrt(
            (pose1.pose.position.x - pose2.pose.position.x)**2 +
            (pose1.pose.position.y - pose2.pose.position.y)**2)

def main():
    rclpy.init()

    executor = rclpy.executors.MultiThreadedExecutor()
    lc_node = SpiralSearcherLC('f_search_pipeline_wp_node')
    executor.add_node(lc_node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        lc_node.destroy_node()


if __name__ == '__main__':
    main()
