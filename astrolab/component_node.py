import time
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import Float64
from simulation_interfaces.srv import Speed # type: ignore
from simulation_interfaces.action import Angle # type: ignore

class ComponentNode(Node): # TODO: Improve action server implementation
    """Class that provides common functionalities for rotating components on the Physical Sunlight Simulator"""

    def __init__(self, name: str, angle: float, speed: float, qos_profile: int, topic_timer_period_sec: float,
                 angle_lower_bound: float, angle_upper_bound, speed_lower_bound: float, speed_upper_bound):
        self.name = name
        self.angle = angle
        self.speed = speed
        self.qos_profile = qos_profile
        
        super().__init__(f'{self.name}_node')
        
        self.angle_topic_publisher = self.create_publisher(Float64, f'{self.name}_angle_topic', qos_profile)
        self.speed_topic_publisher = self.create_publisher(Float64, f'{name}_speed_topic', qos_profile)
        self.speed_service = self.create_service(Speed, f'{name}_speed_service', self.speed_service_callback)
        self.angle_action_server = ActionServer(
            self,
            Angle,
            f'{name}_angle_action',
            self.angle_action_callback)
        self.timer = self.create_timer(topic_timer_period_sec, self.topic_timer_callback)
        self.angle_lower_bound = angle_lower_bound
        self.angle_upper_bound = angle_upper_bound
        self.speed_lower_bound = speed_lower_bound
        self.speed_upper_bound = speed_upper_bound
        
    def topic_timer_callback(self):
        """Publishes the angle and speed topics every period"""
        angle_msg = Float64()
        speed_msg = Float64()
        angle_msg.data = self.angle
        speed_msg.data = self.speed
        self.angle_topic_publisher.publish(angle_msg)
        self.speed_topic_publisher.publish(speed_msg)
        self.get_logger().info(f'[Topic] Published angle = {str(angle_msg.data)}')
        self.get_logger().info(f'[Topic] Published speed = {str(speed_msg.data)}')
        
    def speed_service_callback(self, request, response):
        """Provides responses for speed adjustment services"""
        self.get_logger().info(f'[Service] Received speed adjustment request = {str(self.speed)} -> {str(request.speed)}')
        if (request.speed >= self.speed_lower_bound and request.speed <= self.speed_upper_bound): # In valid range
            self.speed = request.speed
            response.response = True
            self.get_logger().info(f'[Service] Accepted speed adjustment request = {str(request.speed)}')
        else: # Not in valid range
            response.response = False
            self.get_logger().info(f'[Service] Rejected speed adjustment request = {str(request.speed)}')
        return response  
    
    # Temporary measure
    def angle_action_callback(self, goal_handle):
        """Provides responses and feedback for angle adjustment actions"""
        self.get_logger().info(f'[Action] Received angle adjustment request = {str(self.angle)} -> {str(goal_handle.request.requested_angle)}')
        if (goal_handle.request.requested_angle >= self.angle_lower_bound and goal_handle.request.requested_angle <= self.angle_upper_bound): # In valid range
            self.get_logger().info(f'[Service] Accepted angle adjustment request = {str(goal_handle.request.requested_angle)}')
            ITERATIONS = 10
            delta = (goal_handle.request.requested_angle - self.angle) / ITERATIONS
            feedback_msg = Angle.Feedback()
            feedback_msg.current_angle = self.angle

            while self.angle != goal_handle.request.requested_angle:
                feedback_msg.current_angle = self.angle
                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info(f'[Action] Current angle = {self.angle}')
                self.angle += delta
                time.sleep(1)

            goal_handle.succeed()
        else:
            self.get_logger().info(f'[Service] Rejected angle adjustment request = {str(goal_handle.request.requested_angle)}')
            goal_handle.abort()
        
        result = Angle.Result()
        result.final_angle = self.angle
        
        return result
