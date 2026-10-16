
"""Monitor lidar distance and warn about nearby obstacles."""

from collections import deque
from statistics import median

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
        self.declare_parameter('filter_window', 5)

        self.threshold = float(
            self.get_parameter('threshold').value
        )
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.filter_window = int(
            self.get_parameter('filter_window').value
        )

        if self.threshold <= 0:
            raise ValueError('threshold must be positive')

        if self.filter_window < 1:
            raise ValueError('filter_window must be at least 1')

        self.distance_history = deque(maxlen=self.filter_window)
        self.last_state = None

        self.publisher = self.create_publisher(
            Bool, output_topic, 10
        )
        self.subscription = self.create_subscription(
            Range, input_topic, self.range_callback, 10
        )

        self.get_logger().info(
            f'Distance monitor started: threshold={self.threshold:.2f} m, '
            f'filter_window={self.filter_window}, '
            f'input={input_topic}, output={output_topic}'
        )

    def range_callback(self, msg):
        """Filter valid measurements and publish obstacle status."""
        distance = float(msg.range)

        if not distance == distance:
            return

        if distance == float('inf') or distance <= 0:
            return

        if distance < msg.min_range or distance > msg.max_range:
            return

        self.distance_history.append(distance)
        filtered_distance = median(self.distance_history)

        too_close = filtered_distance < self.threshold

        status = Bool()
        status.data = too_close
        self.publisher.publish(status)

        if too_close != self.last_state:
            if too_close:
                self.get_logger().warning(
                    f'TOO CLOSE! Filtered distance: '
                    f'{filtered_distance:.2f} m'
                )
            else:
                self.get_logger().info(
                    f'Distance OK: {filtered_distance:.2f} m'
                )

            self.last_state = too_close


def main(args=None):
    """Run the distance monitor node."""
    rclpy.init(args=args)
    node = None

    try:
        node = DistanceMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
