import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random

class Sensor3(Node):
    def __init__(self):
        super().__init__('sensor3')
        self.publisher = self.create_publisher(Float64, 'sensor_3', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = Float64()
        msg.data = random.uniform(0.0, 10.0)
        self.publisher.publish(msg)
        self.get_logger().info('Sensor 3: "%f"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = Sensor3()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()