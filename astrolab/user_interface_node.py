import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from simulation_interfaces.srv import Speed # type: ignore
from simulation_interfaces.action import Angle # type: ignore

class user_interface_node(Node):
    """Class that provides a node for user_interface"""

    def __init__(self):
        super().__init__('user_interface_node')
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
        self.arm_speed_client = self.create_client(Speed, 'arm_speed_service')
        self.table_speed_client = self.create_client(Speed, 'table_speed_service')
        self.arm_action_client = ActionClient(self, Angle, 'arm_angle_action')
        self.table_action_client = ActionClient(self, Angle, 'table_angle_action')
        
        # prevent unused variable warning
        self.arm_angle_subscription 
        self.arm_speed_subscription
        self.table_angle_subscription 
        self.table_speed_subscription
        
        self.change_arm_speed(42.0)
        self.change_table_speed(42.0)
        
        self.arm_send_goal(10)
        self.table_send_goal(10)
        
    def arm_angle_listener_callback(self, msg):
        self.get_logger().info('arm_angle_listener_callback: "%s"' % msg.data)
        
    def arm_speed_listener_callback(self, msg):
        self.get_logger().info('arm_speed_listener_callback: "%s"' % msg.data)
        
    def table_angle_listener_callback(self, msg):
        self.get_logger().info('table_angle_listener_callback: "%s"' % msg.data)
        
    def table_speed_listener_callback(self, msg):
        self.get_logger().info('table_speed_listener_callback: "%s"' % msg.data)
        
    def change_arm_speed(self, speed):
        while not self.arm_speed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arm_speed_service not available, waiting again...')
    
        request = Speed.Request()
        request.speed = speed
        future = self.arm_speed_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Result of change_arm_speed: "%s" ' % future.result().response)
        
    def change_table_speed(self, speed):
        while not self.table_speed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('table_speed_service not available, waiting again...')
    
        request = Speed.Request()
        request.speed = speed
        future = self.table_speed_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Result of change_table_speed: "%s" ' % future.result().response)
        
    def arm_send_goal(self, angle):
        goal_msg = Angle.Goal()
        goal_msg.requested_angle = angle

        self.arm_action_client.wait_for_server()

        return self.arm_action_client.send_goal_async(goal_msg)
    
    def table_send_goal(self, angle):
        goal_msg = Angle.Goal()
        goal_msg.requested_angle = angle

        self.table_action_client.wait_for_server()

        return self.table_action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)

    node = user_interface_node()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()