import copy
import math

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_srvs.srv import SetBool


MESSAGE_TYPES = {'imu': Imu, 'odometry': Odometry, 'scan': LaserScan, 'twist': Twist}
FAULT_MODES = {'drop', 'zero_stamp', 'bad_frame', 'nonfinite_data'}
HEADER_TYPES = {'imu', 'odometry', 'scan'}


def inject_fault(message, message_type, fault_mode, bad_frame='fault_frame'):
    """Return a faulted copy, or None when the message is intentionally dropped."""
    if message_type not in MESSAGE_TYPES:
        raise ValueError(f'unsupported message_type: {message_type}')
    if fault_mode not in FAULT_MODES:
        raise ValueError(f'unsupported fault_mode: {fault_mode}')
    if fault_mode in {'zero_stamp', 'bad_frame'} and message_type not in HEADER_TYPES:
        raise ValueError(f'{fault_mode} requires a message with a header')
    if fault_mode == 'drop':
        return None

    output = copy.deepcopy(message)
    if fault_mode == 'zero_stamp':
        output.header.stamp.sec = 0
        output.header.stamp.nanosec = 0
    elif fault_mode == 'bad_frame':
        output.header.frame_id = bad_frame
    elif message_type == 'scan':
        output.ranges = list(output.ranges)
        if output.ranges:
            output.ranges[0] = math.nan
        else:
            output.ranges = [math.nan]
    elif message_type == 'imu':
        output.angular_velocity.x = math.nan
    elif message_type == 'odometry':
        output.pose.covariance[0] = math.nan
    else:
        output.linear.x = math.nan
    return output


class FaultInjector(Node):
    def __init__(self):
        super().__init__('fault_injector')
        self.message_type = self.declare_parameter('message_type', 'scan').value
        self.fault_mode = self.declare_parameter('fault_mode', 'drop').value
        self.input_topic = self.declare_parameter('input_topic', '/fault/source').value
        self.output_topic = self.declare_parameter('output_topic', '/fault/output').value
        self.bad_frame = self.declare_parameter('bad_frame', 'fault_frame').value
        self.enabled = bool(self.declare_parameter('initially_enabled', False).value)

        if self.message_type not in MESSAGE_TYPES:
            raise ValueError(f'unsupported message_type: {self.message_type}')
        if self.fault_mode not in FAULT_MODES:
            raise ValueError(f'unsupported fault_mode: {self.fault_mode}')
        if (self.fault_mode in {'zero_stamp', 'bad_frame'}
                and self.message_type not in HEADER_TYPES):
            raise ValueError(f'{self.fault_mode} requires a message with a header')

        message_class = MESSAGE_TYPES[self.message_type]
        qos = qos_profile_sensor_data if self.message_type in {'scan', 'imu'} else QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(message_class, self.output_topic, qos)
        self.subscription = self.create_subscription(
            message_class, self.input_topic, self.receive, qos)
        self.diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.toggle_service = self.create_service(SetBool, '~/enable', self.set_enabled)
        self.received_count = 0
        self.injected_count = 0
        self.forwarded_count = 0
        self.publish_diagnostic('fault injector ready')

    def receive(self, message):
        self.received_count += 1
        output = message
        if self.enabled:
            self.injected_count += 1
            output = inject_fault(message, self.message_type, self.fault_mode, self.bad_frame)
        if output is not None:
            self.publisher.publish(output)
            self.forwarded_count += 1

    def set_enabled(self, request, response):
        self.enabled = request.data
        response.success = True
        response.message = 'fault injection enabled' if self.enabled else 'normal relay restored'
        self.publish_diagnostic(response.message)
        return response

    def publish_diagnostic(self, message):
        status = DiagnosticStatus()
        status.name = f'fault_injection/{self.message_type}'
        status.hardware_id = 'simulation-only'
        status.level = DiagnosticStatus.WARN if self.enabled else DiagnosticStatus.OK
        status.message = message
        status.values = [
            KeyValue(key='enabled', value=str(self.enabled).lower()),
            KeyValue(key='fault_mode', value=self.fault_mode),
            KeyValue(key='input_topic', value=self.input_topic),
            KeyValue(key='output_topic', value=self.output_topic),
            KeyValue(key='received_count', value=str(self.received_count)),
            KeyValue(key='injected_count', value=str(self.injected_count)),
            KeyValue(key='forwarded_count', value=str(self.forwarded_count)),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostics.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = FaultInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
