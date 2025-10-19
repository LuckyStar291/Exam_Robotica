import rclpy
from rclpy.node import Node
from ucb_interfaces.msg import FilteredSensor

class DisplayNode(Node):
    def __init__(self):
        super().__init__('display')
        self.subscription = self.create_subscription(
            FilteredSensor, 'filtered_sensor', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info('Average sensor value: "%f" (Name: %s)' % (msg.sensor_value, msg.name))

def main(args=None):
    rclpy.init(args=args)
    node = DisplayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()