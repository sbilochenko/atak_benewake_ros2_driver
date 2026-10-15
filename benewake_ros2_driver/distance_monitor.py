
"""Monitor lidar distance and warn about nearby obstacles."""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Range
from std_msgs.msg import Bool


class DistanceMonitor(Node):
    """Warn when an obstacle is closer than the configured threshold."""

    def __init__(self):
        """Initialize the distance monitor."""
        super().__init__('distance_monitor')

        self.declare_parameter('threshold', 0.15)
        self.declare_parameter('input_topic', '/lidar/range')
        self.declare_parameter('output_topic', '/lidar/too_close')

        self.threshold = float(
            self.get_parameter('threshold').value
        )
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.last_state = None

        self.publisher = self.create_publisher(
            Bool, output_topic, 10
        )
        self.subscription = self.create_subscription(
            Range, input_topic, self.range_callback, 10
        )

        self.get_logger().info(
            f'Distance monitor started: threshold={self.threshold:.2f} m, '
            f'input={input_topic}, output={output_topic}'
        )

    def range_callback(self, msg):
        """Check the measured distance and publish the warning state."""
        distance = msg.range

        # Ignore invalid measurements.
        if distance != distance or distance == float('inf'):
            return

        too_close = distance < self.threshold

        status = Bool()
        status.data = too_close
        self.publisher.publish(status)

        if too_close != self.last_state:
            if too_close:
                self.get_logger().warning(
                    f'TOO CLOSE! Distance: {distance:.2f} m'
                )
            else:
                self.get_logger().info(
                    f'Distance OK: {distance:.2f} m'
                )

            self.last_state = too_close


def main(args=None):
    """Run the distance monitor node."""
    rclpy.init(args=args)
    node = DistanceMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
