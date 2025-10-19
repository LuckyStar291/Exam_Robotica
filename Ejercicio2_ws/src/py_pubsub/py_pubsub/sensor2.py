import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random

class Sensor2(Node):
    def __init__(self):
        super().__init__('sensor2')
        self.publisher = self.create_publisher(Float64, 'sensor_2', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = Float64()
        msg.data = random.uniform(0.0, 10.0)
        self.publisher.publish(msg)
        self.get_logger().info('Sensor 2: "%f"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = Sensor2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()