import rclpy
from rclpy.node import Node
class HealthReporter(Node):
    def __init__(self):
        super().__init__('health_reporter')
        self._mode = self.declare_parameter('mode', 'sim').value
        self.create_timer(1.0, self._report)
    def _report(self):
        self.get_logger().info(f'health reporter active (mode={self._mode}); hardware control is disabled')
def main(args=None):
    rclpy.init(args=args)
    node = HealthReporter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
