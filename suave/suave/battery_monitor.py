#!/usr/bin/env python
import sys
import rclpy
import threading

from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticArray
from mavros_msgs.msg import State
from std_srvs.srv import SetBool


class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor_node')

        self.declare_parameter('battery_events', [''])
        self.battery_events = self.read_battery_events(
            self.get_parameter('battery_events').value)

        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)

        self.mavros_state_sub = self.create_subscription(
            State, 'mavros/state', self.status_cb, 10)
        
        self.battery_cli = self.create_client(SetBool, 'battery_monitor')
        while not self.battery_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def status_cb(self, msg):
        if msg.mode == 'GUIDED':
            self.last_event_time = self.get_clock().now().to_msg().sec
            self.battery_event_timer = self.create_timer(
                1, self.battery_event_cb)
            self.destroy_subscription(self.mavros_state_sub)

    def read_battery_events(self, events):
        battery_events = []
        if events != '':
            for event in events:
                if event != '':
                    battery_events.append(
                        [e.strip() for e in event.strip('()').split(',')])
        self.get_logger().info('battery event is '+str(battery_events))
        return battery_events

    def battery_event_cb(self):
        current_time = self.get_clock().now().to_msg().sec
        delta_time = current_time - self.last_event_time
        self.get_logger().info('Battery event in '+str(int(self.battery_events[0][0])-delta_time))
        if len(self.battery_events) > 0 and \
           delta_time >= int(self.battery_events[0][0]):

            self.change_battery_status()
            self.last_event_time = self.get_clock().now().to_msg().sec
            self.battery_events.pop(0)

    def change_battery_status(self):
        self.get_logger().info('function designs unavailable due to low battery')
        req = SetBool.Request()
        req.data = True # recharge needed set to True
        try:
            battery_call_response = self.battery_cli.call(req)
        except Exception as e:
            self.logger().info('Request creation failed %r' % (e,))
            return None
        else:
            return battery_call_response
        
def main():
    print("Starting battery monitor node")

    rclpy.init(args=sys.argv)

    battery_monitor = BatteryMonitor()
    rclpy.spin(battery_monitor)

    rclpy.shutdown()
