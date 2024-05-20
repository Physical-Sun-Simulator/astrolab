import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class digital_twin_node(Node):
    """Class that provides a node for the digital twin"""

    def __init__(self):
        super().__init__('digital_twin_node')
        self.arm_angle_subscription = self.create_subscription(
            String,
            'arm_angle_topic',
            self.arm_angle_listener_callback,
            10)
        self.arm_speed_subscription = self.create_subscription(
            String,
            'arm_speed_topic',
            self.arm_speed_listener_callback,
            10)
        self.table_angle_subscription = self.create_subscription(
            String,
            'table_angle_topic',
            self.table_angle_listener_callback,
            10)
        self.table_speed_subscription = self.create_subscription(
            String,
            'table_speed_topic',
            self.table_speed_listener_callback,
            10)
        
        # prevent unused variable warning
        self.arm_angle_subscription 
        self.arm_speed_subscription
        self.table_angle_subscription 
        self.table_speed_subscription
        
    def arm_angle_listener_callback(self, msg):
        self.get_logger().info('arm_angle_listener_callback: "%s"' % msg.data)
        
    def arm_speed_listener_callback(self, msg):
        self.get_logger().info('arm_speed_listener_callback: "%s"' % msg.data)
        
    def table_angle_listener_callback(self, msg):
        self.get_logger().info('table_angle_listener_callback: "%s"' % msg.data)
        
    def table_speed_listener_callback(self, msg):
        self.get_logger().info('table_speed_listener_callback: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    node = digital_twin_node()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()