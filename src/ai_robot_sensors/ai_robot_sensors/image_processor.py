from collections import deque
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def rgb8_to_mono8(message):
    """Convert a row-padded rgb8 image to mono8 while preserving its header."""
    if message.encoding.lower() != 'rgb8':
        raise ValueError(f'expected rgb8 input, got {message.encoding}')
    if message.width <= 0 or message.height <= 0:
        raise ValueError('image dimensions must be positive')
    row_bytes = message.width * 3
    if message.step < row_bytes or len(message.data) < message.step * message.height:
        raise ValueError('rgb8 image data is shorter than its declared layout')

    mono = bytearray(message.width * message.height)
    output_index = 0
    for row in range(message.height):
        offset = row * message.step
        for column in range(message.width):
            pixel = offset + column * 3
            red, green, blue = message.data[pixel:pixel + 3]
            mono[output_index] = (77 * red + 150 * green + 29 * blue) >> 8
            output_index += 1

    output = Image()
    output.header = message.header
    output.height = message.height
    output.width = message.width
    output.encoding = 'mono8'
    output.is_bigendian = 0
    output.step = message.width
    output.data = bytes(mono)
    return output


class ImageProcessor(Node):
    def __init__(self):
        super().__init__('camera_gray_processor')
        self.input_topic = self.declare_parameter(
            'input_topic', '/camera/image_raw').value
        self.output_topic = self.declare_parameter(
            'output_topic', '/camera/image_mono').value
        self.expected_rate = float(self.declare_parameter(
            'expected_rate', 15.0).value)
        self.max_latency_ms = float(self.declare_parameter(
            'max_latency_ms', 100.0).value)
        self.publisher = self.create_publisher(
            Image, self.output_topic, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            Image, self.input_topic, self.receive, qos_profile_sensor_data)
        self.diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.receive_times = deque(maxlen=101)
        self.latencies_ms = deque(maxlen=101)
        self.last_stamp = None
        self.last_error = ''
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(
            1.0, self.publish_diagnostic, clock=self.steady_clock)

    def receive(self, message):
        started = time.monotonic()
        try:
            output = rgb8_to_mono8(message)
            self.publisher.publish(output)
            self.last_stamp = output.header.stamp
            self.last_error = ''
        except ValueError as error:
            self.last_error = str(error)
            return
        self.receive_times.append(started)
        self.latencies_ms.append((time.monotonic() - started) * 1000.0)

    def measured_rate(self):
        if len(self.receive_times) < 3:
            return 0.0
        duration = self.receive_times[-1] - self.receive_times[0]
        return 0.0 if duration <= 0.0 else (len(self.receive_times) - 1) / duration

    def publish_diagnostic(self):
        rate = self.measured_rate()
        latency = max(self.latencies_ms, default=float('inf'))
        rate_valid = 0.7 * self.expected_rate <= rate <= 1.3 * self.expected_rate
        latency_valid = latency <= self.max_latency_ms
        status = DiagnosticStatus()
        status.name = 'perception/camera_gray'
        status.hardware_id = 'software'
        if self.last_error:
            status.level = DiagnosticStatus.ERROR
            status.message = self.last_error
        elif not self.latencies_ms:
            status.level = DiagnosticStatus.STALE
            status.message = 'no image processed'
        elif not rate_valid or not latency_valid:
            status.level = DiagnosticStatus.WARN
            status.message = 'camera processing outside tolerance'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'camera grayscale processing valid'
        status.values = [
            KeyValue(key='input_topic', value=self.input_topic),
            KeyValue(key='output_topic', value=self.output_topic),
            KeyValue(key='output_encoding', value='mono8'),
            KeyValue(key='measured_rate_hz', value=f'{rate:.3f}'),
            KeyValue(key='max_latency_ms', value=f'{latency:.3f}'),
        ]
        array = DiagnosticArray()
        if self.last_stamp is not None:
            array.header.stamp = self.last_stamp
        array.status = [status]
        self.diagnostics.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        except RuntimeError:
            if rclpy.ok():
                raise
        if rclpy.ok():
            rclpy.shutdown()
