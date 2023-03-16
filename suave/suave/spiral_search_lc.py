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

from geometry_msgs.msg import PoseStamped


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
        self.spiral_count: int = 1
        self.spiral_x: float = 0.0
        self.spiral_y: float = 0.0
        self.timer_period = 1.0
        self._timer: Optional[Timer] = None
        self.count = 0
        self.z_delta = 0
        self.old_spiral_altitude = -1

        super().__init__(node_name, **kwargs)

        self.goal_setpoint = PoseStamped()
        self.goal_setpoint_local = PoseStamped()
        self.goal_setpoint_local.pose.position.x = -100.0
        self.goal_setpoint_local.pose.position.y = -100.0
        self.goal_setpoint_local.pose.position.z = -100.0

        self.xyz = None
        self.spiral_count_added = False

        spiral_altitude_descriptor = ParameterDescriptor(
            description='Sets the spiral altitude of the UUV.')
        self.declare_parameter(
            'spiral_altitude', 2.0, spiral_altitude_descriptor)

        self.ardusub = BlueROVGazebo('bluerov_spiral_search')
        self.thread = threading.Thread(
            target=rclpy.spin, args=(self.ardusub, ), daemon=True)
        self.thread.start()

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

    def publish(self):
        if self._enabled is True:
            self.spiral_altitude = self.get_parameter(
                    'spiral_altitude').get_parameter_value().double_value
            self.get_logger().info('Spiral altitude is ' + str(self.spiral_altitude))

            fov = math.pi/3
            pipe_z = 0.5
            spiral_width = 2.0*self.spiral_altitude*math.tan(fov/2)

            if self.count > 10:
                # self.ardusub.altitude = self.spiral_altitude + self.z_delta
                # self.ardusub.setpoint_position_local(
                #     self.spiral_x, self.spiral_y, fixed_altitude=True)
                self.setpoint_pub.publish(self.goal_setpoint)
                self.get_logger().info('published x='+str(self.goal_setpoint.pose.position.x)+
                    ', y='+str(self.goal_setpoint.pose.position.y)+', z='+str(self.goal_setpoint.pose.position.z))
                self.count = 0
                self.spiral_count+=1
                self.spiral_count_added = True

            if self.spiral_count_added or self.xyz == None or\
               self.ardusub.check_setpoint_reached_xy(self.goal_setpoint_local, 0.4):
                self.spiral_count_added = False
                self.get_logger().info('check setpoint reached is ')
                self.get_logger().info(str(self.ardusub.check_setpoint_reached_xy(self.goal_setpoint_local, 0.4)))

                x, y = spiral_points(
                    self.spiral_count,
                    self.spiral_x,
                    self.spiral_y,
                    resolution=0.1,
                    spiral_width=spiral_width)
                self.get_logger().info(
                        'setpoint_postion_local value {0}, {1}'.format(x, y))

                self.ardusub.altitude = self.spiral_altitude
                self.goal_setpoint.pose.position.x = x
                self.goal_setpoint.pose.position.y = y
                self.goal_setpoint.pose.position.z = self.spiral_altitude
                self.xyz = self.ardusub.calc_position_local(x,y,fixed_altitude=True)
                if self.xyz != None:
                    self.goal_setpoint_local.pose.position.x = self.xyz[0]
                    self.goal_setpoint_local.pose.position.y = self.xyz[1]
                    self.goal_setpoint_local.pose.position.z = self.xyz[2]
                    self.setpoint_pub.publish(self.goal_setpoint)
                    self.get_logger().info('published x='+
                        str(self.goal_setpoint.pose.position.x)+', y='+str(self.goal_setpoint.pose.position.y)+
                        ', z='+str(self.goal_setpoint.pose.position.z))
                    self.spiral_count += 1
                    self.spiral_x = x
                    self.spiral_y = y
                    self.count = 0
            else:
                self.count += 1

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_configure() is called.')

        self.setpoint_pub = self.create_publisher(
            PoseStamped, 'maintain_motion/setpoint', 10)

        self._timer_ = self.create_timer(self.timer_period, self.publish)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate() is called.")
        self._enabled = True
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
