import rclpy, math
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float64
from simulation_interfaces.srv import Speed # type: ignore
from simulation_interfaces.action import Angle # type: ignore


class user_interface_node(Node):
    """Class that provides a node for user_interface"""

    def __init__(self):
        super().__init__('user_interface_node')
        self.arm_angle_subscription = self.create_subscription(
            Float64,
            'arm_angle_topic',
            self.arm_angle_listener_callback,
            10)
        self.arm_speed_subscription = self.create_subscription(
            Float64,
            'arm_speed_topic',
            self.arm_speed_listener_callback,
            10)
        self.table_angle_subscription = self.create_subscription(
            Float64,
            'table_angle_topic',
            self.table_angle_listener_callback,
            10)
        self.table_speed_subscription = self.create_subscription(
            Float64,
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
        
    def arm_angle_listener_callback(self, msg):
        self.get_logger().info(f'[Topic]: Read arm angle = {msg.data}')
        
    def arm_speed_listener_callback(self, msg):
        self.get_logger().info(f'[Topic]: Read arm speed = {msg.data}')
        
    def table_angle_listener_callback(self, msg):
        self.get_logger().info(f'[Topic]: Read table angle = {msg.data}')
        
    def table_speed_listener_callback(self, msg):
        self.get_logger().info(f'[Topic]: Read table speed = {msg.data}')
        
    def change_arm_speed(self, speed):
        while not self.arm_speed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('[Service] Waiting for arm_speed_service...')
    
        request = Speed.Request()
        request.speed = speed
        future = self.arm_speed_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f'[Service] Changed arm speed to {speed}')
        
    def change_table_speed(self, speed):
        while not self.table_speed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('[Service] Waiting for table_speed_service...')
    
        request = Speed.Request()
        request.speed = speed
        future = self.table_speed_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f'[Service] Changed table speed to {speed}')
        
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