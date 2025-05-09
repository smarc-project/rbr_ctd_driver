import rclpy
from rclpy.node import Node
from rbr_ctd_interfaces.msg import RBRCTD, Topics
import rclpy.time
from std_msgs.msg import String

class CTDDecoder(Node):
    # Example data:
    # 2000-01-01 00:04:27.000, 0.0029, 21.7070, 10.2192, 0.0867, 0.0860, 0.0110, 1.0000, 22.0666
    
    def __init__(self):
        super().__init__('rbr_ctd_driver')
        self.get_logger().info("Starting RBR CTD driver node to decode raw CTD data")
        self.publisher = self.create_publisher(RBRCTD, Topics.CTD_TOPIC, 10)
        self.subscriber = self.create_subscription(
            String,
            Topics.CTD_RAW_TOPIC,
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f"Received message: {msg.data}")
        msg = msg.data.strip('\r\n').split(',')
        if len(msg) != 9:
            self.get_logger().error(f"Invalid message format. Length should be 9, got {len(msg)}")
            return
        ctd_msg = RBRCTD()
        ctd_msg.time = self.get_clock().now().to_msg()
        ctd_msg.time_ctd = msg[0]
        ctd_msg.conductivity = float(msg[1])
        ctd_msg.temperature = float(msg[2])
        ctd_msg.pressure = float(msg[3])
        ctd_msg.sea_pressure = float(msg[4])
        ctd_msg.depth = float(msg[5])
        ctd_msg.salinity = float(msg[6])
        ctd_msg.samples = float(msg[7])
        ctd_msg.temperature_conductivity_correction = float(msg[8])

        self.get_logger().info(f"Publishing message: {ctd_msg}")
        self.publisher.publish(ctd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CTDDecoder()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()