import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.clock import Clock, ClockType
from rclpy.node import Node


def build_status(mode):
    status = DiagnosticStatus()
    status.level = DiagnosticStatus.OK
    status.name = 'ai_robot_tools/health_reporter'
    status.message = 'foundation active; hardware control is disabled'
    status.hardware_id = 'foundation'
    status.values = [
        KeyValue(key='mode', value=mode),
        KeyValue(key='hardware_control', value='disabled'),
    ]
    return status


class HealthReporter(Node):
    def __init__(self):
        super().__init__('health_reporter')
        self._mode = self.declare_parameter('mode', 'sim').value
        self._publisher = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)
        self._timer_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0, self._report, clock=self._timer_clock)

    def _report(self):
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [build_status(self._mode)]
        self._publisher.publish(message)
        self.get_logger().info(f'health reporter active (mode={self._mode}); hardware control is disabled')


def main(args=None):
    rclpy.init(args=args)
    node = HealthReporter()
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
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass
if __name__ == '__main__':
    main()
