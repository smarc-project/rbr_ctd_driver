import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rbr_ctd_interfaces.msg import Topics
import time


class SerialReaderCTD(Node):
    """
    A ROS2 node that reads data from a serial port and publishes it to a topic.
    """

    def __init__(self):
        super().__init__("serial_reader_ctd")
        self.get_logger().info("Setting up serial reader for RBR CTD")

        self.declare_parameter("port", "/dev/ttyUSB1")
        self.declare_parameter("baudrate", 115200)
        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self.publisher = self.create_publisher(String, Topics.CTD_RAW_TOPIC, 10)

    def start_serial_reader(self):
        """
        Start the serial reader in a separate thread.
        This method will keep trying to connect until successful or the node is shut down.
        """
        self.get_logger().info("Starting serial reader for RBR CTD. Block until connected.")
        self._connect_serial()
        self.read_serial_and_publish()

    def _connect_serial(self):
        """
        Connect to the serial port if not already connected.
        This method will keep trying to connect until successful or the node is shut down.
        """
        self.connected = False
        sleep_time = .5
        while not self.connected and rclpy.ok():
            try:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
                self.connected = True
            except serial.SerialException as e:
                sleep_time = min(sleep_time * 2, 30)  # Exponential backoff but cap at 30 seconds
                self.get_logger().info(f"Error opening serial port: {e}. Trying again in {sleep_time} seconds.")
                time.sleep(sleep_time)

    def read_serial_and_publish(self):
        """
        Read a line from the serial port and publish it.
        Loop until the node is shut down or the serial port is closed.
        """
        while self.connected and self.ser.is_open:
            line = self.ser.readline().decode("utf-8").strip()
            if len(line) > 0:
                msg = String()
                msg.data = line
                self.publisher.publish(msg)
                self.get_logger().info(f"Publishing: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = SerialReaderCTD()
    try:
        node.start_serial_reader()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down serial reader")
    finally:
        node.ser.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
