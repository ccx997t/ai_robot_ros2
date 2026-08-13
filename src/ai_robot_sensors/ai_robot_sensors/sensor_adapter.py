import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan


MESSAGE_TYPES = {
    'camera_info': CameraInfo,
    'image': Image,
    'imu': Imu,
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
        self.last_received = None
        self.message_count = 0
        message_type = MESSAGE_TYPES[sensor_type]
        self.publisher = self.create_publisher(
            message_type, self.output_topic, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            message_type, self.input_topic, self.receive, qos_profile_sensor_data)
        self.diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.timer = self.create_timer(1.0, self.publish_diagnostic)

    def receive(self, message):
        message.header.frame_id = self.frame_id
        self.publisher.publish(message)
        self.last_received = time.monotonic()
        self.message_count += 1

    def publish_diagnostic(self):
        now = time.monotonic()
        age = float('inf') if self.last_received is None else now - self.last_received
        stale_after = max(1.0, 3.0 / self.expected_rate)
        status = DiagnosticStatus()
        status.name = f'sensors/{self.sensor_type}'
        status.hardware_id = 'gazebo'
        if age > stale_after:
            status.level = DiagnosticStatus.STALE
            status.message = 'sensor data stale or not received'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'sensor stream active'
        status.values = [
            KeyValue(key='topic', value=self.output_topic),
            KeyValue(key='frame_id', value=self.frame_id),
            KeyValue(key='message_count', value=str(self.message_count)),
            KeyValue(key='age_seconds', value='inf' if age == float('inf') else f'{age:.3f}'),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
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
