import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from ucb_interfaces.msg import FilteredSensor

class FilterNode(Node):
    def __init__(self):
        super().__init__('filter')
        self.sensor1_value = 0.0
        self.sensor2_value = 0.0
        self.sensor3_value = 0.0
        self.received_count = 0

        # Suscripciones a los tópicos de los sensores
        self.subscription1 = self.create_subscription(
            Float64, 'sensor_1', self.sensor1_callback, 10)
        self.subscription2 = self.create_subscription(
            Float64, 'sensor_2', self.sensor2_callback, 10)
        self.subscription3 = self.create_subscription(
            Float64, 'sensor_3', self.sensor3_callback, 10)

        # Publicador para el tópico filtrado
        self.publisher = self.create_publisher(FilteredSensor, 'filtered_sensor', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def sensor1_callback(self, msg):
        self.sensor1_value = msg.data
        self.received_count += 1

    def sensor2_callback(self, msg):
        self.sensor2_value = msg.data
        self.received_count += 1

    def sensor3_callback(self, msg):
        self.sensor3_value = msg.data
        self.received_count += 1

    def timer_callback(self):
        if self.received_count == 3:  # Asegura que se hayan recibido los 3 valores
            avg = (self.sensor1_value + self.sensor2_value + self.sensor3_value) / 3.0
            msg = FilteredSensor()
            msg.sensor_value = avg
            msg.name = "sensor_avg"
            self.publisher.publish(msg)
            self.get_logger().info('Filtered average: "%f"' % avg)
            self.received_count = 0  # Reinicia el contador

def main(args=None):
    rclpy.init(args=args)
    node = FilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()