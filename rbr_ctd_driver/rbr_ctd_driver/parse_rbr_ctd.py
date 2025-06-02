import rclpy
from rclpy.node import Node
from rbr_ctd_interfaces.msg import RBRCTD
from std_msgs.msg import String


def parse_ctd_data(data: str) -> RBRCTD:
    """
    Parse the raw CTD data string into a RBRCTD message.
    """
    data = data.strip('\r\n').split(',')
    if len(data) != 9:
        raise ValueError(f"Invalid message format. Length should be 9, got {len(data)}")

    ctd_data = RBRCTD()
    ctd_data.time_ctd = data[0]
    ctd_data.conductivity = float(data[1])
    ctd_data.temperature = float(data[2])
    ctd_data.pressure = float(data[3])
    ctd_data.sea_pressure = float(data[4])
    ctd_data.depth = float(data[5])
    ctd_data.salinity = float(data[6])
    ctd_data.samples = float(data[7])
    ctd_data.temperature_conductivity_correction = float(data[8])
    return ctd_data

class CTDDecoder(Node):
    # Example data:
    # 2000-01-01 00:04:27.000, 0.0029, 21.7070, 10.2192, 0.0867, 0.0860, 0.0110, 1.0000, 22.0666
    
    def __init__(self):
        super().__init__('rbr_ctd_driver')
        self.get_logger().info("Starting RBR CTD driver node to decode raw CTD data")
        self.input_topic = self.declare_parameter("input_topic", "input_topic").value
        self.output_topic = self.declare_parameter("output_topic", "output_topic").value
        print(f"Input topic: {self.input_topic}, Output topic: {self.output_topic}")
        self.publisher = self.create_publisher(RBRCTD, self.output_topic, 10)
        self.subscriber = self.create_subscription(
            String,
            self.input_topic,
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f"Received message: {msg.data}")
        try:
            ctd_data = parse_ctd_data(msg.data)
            ctd_data.time = self.get_clock().now().to_msg()
            self.get_logger().info(f"Publishing messaage: {ctd_data}")
            self.publisher.publish(ctd_data)
        except ValueError as e:
            self.get_logger().error(f"Error parsing data: {e}")
            return

def main(args=None):
    rclpy.init(args=args)
    node = CTDDecoder()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()