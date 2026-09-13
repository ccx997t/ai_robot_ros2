import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from .reactive_avoidance import AvoidanceConfig, ReactiveAvoidance


class ReactiveAvoidanceNode(Node):
    def __init__(self):
        super().__init__('reactive_avoidance')
        config = AvoidanceConfig(
            forward_speed=float(self.declare_parameter('forward_speed', 0.10).value),
            turn_speed=float(self.declare_parameter('turn_speed', 0.45).value),
            stop_distance=float(self.declare_parameter('stop_distance', 0.70).value),
            clear_distance=float(self.declare_parameter('clear_distance', 1.00).value),
        )
        self.scan_timeout = float(self.declare_parameter('scan_timeout', 0.5).value)
        if not 0.0 < self.scan_timeout <= 2.0:
            raise ValueError('scan_timeout must be in (0.0, 2.0]')
        self.behavior = ReactiveAvoidance(config)
        self.last_scan = None
        self.last_scan_time = None
        self.command_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.receive_scan, qos_profile_sensor_data)
        self.timer = self.create_timer(0.1, self.control)
        self.last_state = 'waiting_for_scan'
        self.get_logger().info('reactive obstacle avoidance active; output=/cmd_vel')

    def receive_scan(self, message):
        self.last_scan = message
        self.last_scan_time = time.monotonic()

    def control(self):
        age = float('inf') if self.last_scan_time is None else (
            time.monotonic() - self.last_scan_time)
        command = Twist()
        level = DiagnosticStatus.STALE
        detail = 'laser scan missing or stale; stop commanded'
        if self.last_scan is not None and age <= self.scan_timeout:
            decision = self.behavior.decide(
                self.last_scan.ranges, self.last_scan.angle_min,
                self.last_scan.angle_increment, self.last_scan.range_max)
            command.linear.x = decision.linear_x
            command.angular.z = decision.angular_z
            self.last_state = decision.state
            level = DiagnosticStatus.OK
            detail = (
                f'{decision.state}; front={decision.front_distance:.2f}m, '
                f'left={decision.left_distance:.2f}m, '
                f'right={decision.right_distance:.2f}m')
        else:
            self.last_state = 'stopped_stale_scan'
        self.command_pub.publish(command)
        self.publish_diagnostic(level, detail, age)

    def publish_diagnostic(self, level, message, scan_age):
        status = DiagnosticStatus()
        status.level = level
        status.name = 'behavior/reactive_avoidance'
        status.hardware_id = 'simulation'
        status.message = message
        status.values = [
            KeyValue(key='state', value=self.last_state),
            KeyValue(key='scan_age_seconds', value=(
                'inf' if scan_age == float('inf') else f'{scan_age:.3f}')),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostic_pub.publish(array)

    def destroy_node(self):
        self.command_pub.publish(Twist())
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
