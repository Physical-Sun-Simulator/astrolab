import time, rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import String
from simulation_interfaces.srv import Speed # type: ignore
from simulation_interfaces.action import Angle # type: ignore

class arm_node(Node):
    """Class that provides a node for the arm"""

    def __init__(self):
        super().__init__('arm_node')
        self.angle = 0
        self.speed = 0
        self.angle_publisher = self.create_publisher(String, 'arm_angle_topic', 10)
        self.speed_publisher = self.create_publisher(String, 'arm_speed_topic', 10)
        self.speed_service = self.create_service(Speed, 'arm_speed_service', self.speed_service_callback)
        self._action_server = ActionServer(
            self,
            Angle,
            'arm_angle_action',
            self.angle_action_callback)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.callback_counter = 0
        
    def timer_callback(self):
        angle_msg = String()
        speed_msg = String()
        angle_msg.data = '[%d] Arm Angle = %d' % (self.callback_counter, self.angle)
        speed_msg.data = '[%d] Arm Speed = %d' % (self.callback_counter, self.speed)
        self.angle_publisher.publish(angle_msg)
        self.speed_publisher.publish(speed_msg)
        self.get_logger().info('Publishing: "%s"' % angle_msg.data)
        self.get_logger().info('Publishing: "%s"' % speed_msg.data)
        self.callback_counter += 1
        
    def speed_service_callback(self, request, response):
        response.response = 'Adjusted arm speed to %f' % request.speed
        self.get_logger().info('Incoming arm request\nspeed: %f' % request.speed)

        return response
    
    def angle_action_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Angle.Feedback()
        feedback_msg.current_angle = self.angle

        for i in range(1, goal_handle.request.requested_angle):
            feedback_msg.current_angle = i + 1
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.current_angle))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()

        result = Angle.Result()
        result.final_angle = feedback_msg.current_angle
        return result

def main(args=None):
    rclpy.init(args=args)

    node = arm_node()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()