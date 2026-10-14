
"""ROS 2 driver for Benewake lidar sensors."""

from benewake_ros2_driver.parser_factory import create_parser

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Range

import serial


class TFminiSDriver(Node):
    """Publish lidar measurements and recover UART connections."""

    def __init__(self):
        """Initialize the driver, parameters, and timers."""
        super().__init__('benewake_lidar_node')

        self.declare_parameter('model', 'tfmini_s')
        self.declare_parameter('port', '/dev/serial0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'tfmini_s_link')
        self.declare_parameter('topic', '/lidar/range')
        self.declare_parameter('min_range', 0.1)
        self.declare_parameter('max_range', 12.0)
        self.declare_parameter('field_of_view', 0.04)
        self.declare_parameter('reconnect_interval', 2.0)

        model = self.get_parameter('model').value
        self.port = self.get_parameter('port').value
        self.baudrate = int(
            self.get_parameter('baudrate').value
        )
        self.frame_id = self.get_parameter('frame_id').value
        topic = self.get_parameter('topic').value
        self.min_range = float(
            self.get_parameter('min_range').value
        )
        self.max_range = float(
            self.get_parameter('max_range').value
        )
        self.field_of_view = float(
            self.get_parameter('field_of_view').value
        )
        self.reconnect_interval = float(
            self.get_parameter('reconnect_interval').value
        )

        if self.min_range < 0 or self.max_range <= self.min_range:
            raise ValueError(
                'Range limits must satisfy '
                '0 <= min_range < max_range'
            )

        if not 0 <= self.field_of_view <= 6.2832:
            raise ValueError(
                'field_of_view must be between 0 and 2*pi radians'
            )

        if self.reconnect_interval <= 0:
            raise ValueError('reconnect_interval must be positive')

        self.parser = create_parser(model)
        self.publisher = self.create_publisher(Range, topic, 10)
        self.serial_port = None
        self._last_connection_attempt = 0.0

        self.timer = self.create_timer(0.01, self.read_sensor)

        self.get_logger().info(
            f'Benewake lidar initialized: model={model}, '
            f'port={self.port}, baudrate={self.baudrate}, '
            f'topic={topic}'
        )

        self._connect_serial()

    def _connect_serial(self):
        """Attempt to open the UART connection."""
        if self.serial_port is not None:
            return self.serial_port.is_open

        try:
            self.serial_port = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.05,
            )
            self.parser = create_parser(
                self.get_parameter('model').value
            )
            self.get_logger().info(
                f'UART connected: {self.port} '
                f'at {self.baudrate} baud'
            )
            return True
        except (serial.SerialException, OSError) as exc:
            self.serial_port = None
            self.get_logger().warning(
                f'Cannot open UART {self.port}: {exc}. '
                'Will retry.'
            )
            return False

    def _disconnect_serial(self, reason):
        """Close a failed UART connection."""
        self.get_logger().warning(
            f'UART connection lost: {reason}. Will reconnect.'
        )

        if self.serial_port is not None:
            try:
                if self.serial_port.is_open:
                    self.serial_port.close()
            except (serial.SerialException, OSError):
                pass

        self.serial_port = None

    def read_sensor(self):
        """Read measurements and publish valid ranges."""
        if self.serial_port is None:
            now = self.get_clock().now().nanoseconds / 1e9

            if now - self._last_connection_attempt < (
                self.reconnect_interval
            ):
                return

            self._last_connection_attempt = now
            self._connect_serial()
            return

        try:
            waiting = self.serial_port.in_waiting
            data = self.serial_port.read(max(1, waiting))
        except (serial.SerialException, OSError) as exc:
            self._disconnect_serial(exc)
            return

        for measurement in self.parser.feed(data):
            distance_m = measurement.distance_m

            if not self.min_range <= distance_m <= self.max_range:
                continue

            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.radiation_type = Range.INFRARED
            msg.field_of_view = self.field_of_view
            msg.min_range = self.min_range
            msg.max_range = self.max_range
            msg.range = float(distance_m)

            self.publisher.publish(msg)

    def destroy_node(self):
        """Close UART before destroying the ROS 2 node."""
        if self.serial_port is not None:
            try:
                if self.serial_port.is_open:
                    self.serial_port.close()
            except (serial.SerialException, OSError) as exc:
                self.get_logger().warning(
                    f'Error closing UART: {exc}'
                )

        super().destroy_node()


def main(args=None):
    """Run the Benewake lidar node."""
    rclpy.init(args=args)
    node = None

    try:
        node = TFminiSDriver()
        rclpy.spin(node)
    except (serial.SerialException, OSError, ValueError) as exc:
        print(f'Failed to start Benewake lidar: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
