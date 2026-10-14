
import serial

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

from benewake_ros2_driver.protocol import TFminiSParser


class TFminiSDriver(Node):
    def __init__(self):
        super().__init__('tfmini_s_driver')

        self.declare_parameter('port', '/dev/serial0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'tfmini_s_link')

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value

        self.parser = TFminiSParser()
        self.publisher = self.create_publisher(
            Range, '/lidar/range', 10
        )

        self.serial_port = serial.Serial(
            port, baudrate, timeout=0.05
        )

        self.timer = self.create_timer(0.01, self.read_sensor)
        self.get_logger().info(
            f'TFmini-S driver started on {port} at {baudrate} baud'
        )

    def read_sensor(self):
        try:
            waiting = self.serial_port.in_waiting
            data = self.serial_port.read(max(1, waiting))
        except serial.SerialException as exc:
            self.get_logger().error(f'Serial read failed: {exc}')
            return

        for measurement in self.parser.feed(data):
            distance_m = measurement.distance_cm / 100.0

            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.radiation_type = Range.INFRARED
            msg.field_of_view = 0.04
            msg.min_range = 0.1
            msg.max_range = 12.0
            msg.range = float(distance_m)

            if msg.min_range <= msg.range <= msg.max_range:
                self.publisher.publish(msg)

    def destroy_node(self):
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TFminiSDriver()
        rclpy.spin(node)
    except (serial.SerialException, OSError) as exc:
        print(f'Failed to start TFmini-S driver: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
