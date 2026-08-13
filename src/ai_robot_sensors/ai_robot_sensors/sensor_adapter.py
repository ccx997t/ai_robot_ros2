from collections import deque
import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan


MESSAGE_TYPES = {
    'camera_info': CameraInfo,
    'image': Image,
    'imu': Imu,
    'joint_state': JointState,
    'scan': LaserScan,
}


class SensorAdapter(Node):
    def __init__(self):
        super().__init__('sensor_adapter')
        sensor_type = self.declare_parameter('sensor_type', 'scan').value
        if sensor_type not in MESSAGE_TYPES:
            raise ValueError(f'unsupported sensor_type: {sensor_type}')
        self.sensor_type = sensor_type
        self.input_topic = self.declare_parameter('input_topic', '/sim/scan').value
        self.output_topic = self.declare_parameter('output_topic', '/scan').value
        self.frame_id = self.declare_parameter('frame_id', 'laser_link').value
        self.expected_rate = float(self.declare_parameter('expected_rate', 10.0).value)
        self.diagnostic_name = self.declare_parameter(
            'diagnostic_name', sensor_type).value
        self.monitor_only = bool(self.declare_parameter('monitor_only', False).value)
        self.required_joints = list(self.declare_parameter(
            'required_joints', ['left_wheel_joint', 'right_wheel_joint']).value)
        self.last_received = None
        self.receive_times = deque(maxlen=101)
        self.last_stamp = None
        self.timestamp_valid = False
        self.frame_valid = False
        self.data_valid = False
        self.message_count = 0
        message_type = MESSAGE_TYPES[sensor_type]
        self.publisher = None if self.monitor_only else self.create_publisher(
            message_type, self.output_topic, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            message_type, self.input_topic, self.receive, qos_profile_sensor_data)
        self.diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(
            1.0, self.publish_diagnostic, clock=self.steady_clock)

    def receive(self, message):
        if not self.monitor_only:
            message.header.frame_id = self.frame_id
            self.publisher.publish(message)
        now = time.monotonic()
        stamp = (message.header.stamp.sec, message.header.stamp.nanosec)
        self.timestamp_valid = stamp != (0, 0) and (
            self.last_stamp is None or stamp >= self.last_stamp)
        self.last_stamp = stamp
        self.frame_valid = message.header.frame_id == self.frame_id
        self.data_valid = self.validate_data(message)
        self.last_received = now
        self.receive_times.append(now)
        self.message_count += 1

    def validate_data(self, message):
        if self.sensor_type == 'scan':
            return (len(message.ranges) > 0 and message.angle_max > message.angle_min
                    and message.range_max > message.range_min
                    and all(not math.isnan(value) for value in message.ranges))
        if self.sensor_type == 'image':
            return (message.width > 0 and message.height > 0 and bool(message.encoding)
                    and message.step > 0
                    and len(message.data) >= message.step * message.height)
        if self.sensor_type == 'camera_info':
            return (message.width > 0 and message.height > 0
                    and len(message.k) == 9 and message.k[0] > 0.0
                    and message.k[4] > 0.0
                    and all(math.isfinite(value) for value in message.k))
        if self.sensor_type == 'imu':
            values = [
                message.orientation.x, message.orientation.y,
                message.orientation.z, message.orientation.w,
                message.angular_velocity.x, message.angular_velocity.y,
                message.angular_velocity.z, message.linear_acceleration.x,
                message.linear_acceleration.y, message.linear_acceleration.z,
            ]
            quaternion_norm = math.sqrt(sum(value * value for value in values[:4]))
            return all(math.isfinite(value) for value in values) and quaternion_norm > 0.0
        names = set(message.name)
        values_finite = all(math.isfinite(value) for value in message.position)
        values_finite = values_finite and all(
            math.isfinite(value) for value in message.velocity)
        return (set(self.required_joints).issubset(names)
                and len(message.position) == len(message.name)
                and len(message.velocity) in (0, len(message.name))
                and values_finite)

    def measured_rate(self):
        if len(self.receive_times) < 3:
            return 0.0
        duration = self.receive_times[-1] - self.receive_times[0]
        return 0.0 if duration <= 0.0 else (len(self.receive_times) - 1) / duration

    def publish_diagnostic(self):
        now = time.monotonic()
        age = float('inf') if self.last_received is None else now - self.last_received
        stale_after = max(1.0, 3.0 / self.expected_rate)
        measured_rate = self.measured_rate()
        rate_valid = (0.5 * self.expected_rate <= measured_rate
                      <= 1.5 * self.expected_rate)
        status = DiagnosticStatus()
        status.name = f'sensors/{self.diagnostic_name}'
        status.hardware_id = 'gazebo'
        if age > stale_after:
            status.level = DiagnosticStatus.STALE
            status.message = 'sensor data stale or not received'
        elif not self.timestamp_valid or not self.frame_valid or not self.data_valid:
            status.level = DiagnosticStatus.ERROR
            status.message = 'sensor contract invalid'
        elif not rate_valid:
            status.level = DiagnosticStatus.WARN
            status.message = 'sensor frequency outside tolerance'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'sensor contract valid'
        status.values = [
            KeyValue(key='topic', value=(self.input_topic if self.monitor_only
                                         else self.output_topic)),
            KeyValue(key='expected_frame_id', value=self.frame_id),
            KeyValue(key='frame_valid', value=str(self.frame_valid).lower()),
            KeyValue(key='timestamp_valid', value=str(self.timestamp_valid).lower()),
            KeyValue(key='data_valid', value=str(self.data_valid).lower()),
            KeyValue(key='expected_rate_hz', value=f'{self.expected_rate:.3f}'),
            KeyValue(key='measured_rate_hz', value=f'{measured_rate:.3f}'),
            KeyValue(key='rate_valid', value=str(rate_valid).lower()),
            KeyValue(key='message_count', value=str(self.message_count)),
            KeyValue(key='age_seconds', value='inf' if age == float('inf') else f'{age:.3f}'),
        ]
        array = DiagnosticArray()
        if self.last_stamp is not None:
            array.header.stamp.sec = self.last_stamp[0]
            array.header.stamp.nanosec = self.last_stamp[1]
        array.status = [status]
        self.diagnostics.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = SensorAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
