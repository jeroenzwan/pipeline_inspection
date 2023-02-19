import rclpy
import sys

from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import Node
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult


from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import SetParameters


class MaintainMotionLC(Node):
    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)

        speed_descriptor = ParameterDescriptor(
            description='Sets the speed of the UUV.')
        self.declare_parameter(
            'speed', 2.0, speed_descriptor)

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
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate() is called.")
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate() is called.")
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_cleanup() is called.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_shutdown() is called.')
        return TransitionCallbackReturn.SUCCESS

    def call_service(self, srv_type, srv_name, request):
        service = self.create_client(srv_type, srv_name)
        while not service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(srv_name + ' not available, waiting...')
        future = service.call_async(request)
        return future



def main():
    rclpy.init(args=sys.argv)
    maintain_motion_node = MaintainMotionLC('f_maintain_motion_node')
    mt_executor = MultiThreadedExecutor()
    rclpy.spin(maintain_motion_node, mt_executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
