import time, threading
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float64
from simulation_interfaces.srv import Speed, Initialization  # type: ignore
from simulation_interfaces.action import Angle  # type: ignore

# Constants
NODE_NAME = "{name}_node"
ANGLE_TOPIC_NAME = "{name}_angle_topic"
SPEED_TOPIC_NAME = "{name}_speed_topic"
SPEED_SERVICE_NAME = "{name}_speed_service"
INITIALIZE_SERVICE_NAME = "{name}_initialize_service"
ANGLE_ACTION_NAME = "{name}_angle_action"


class ComponentNode(Node):  # TODO: Improve action server implementation
    """Class that provides common functionalities for rotating components on the Physical Sunlight Simulator"""

    def __init__(
        self,
        name: str,
        angle: float,
        angle_lower_bound: float,
        angle_upper_bound: float,
        speed: float,
        speed_lower_bound: float,
        speed_upper_bound,
        qos_profile: int,
    ):
        # Object variables
        self.name = name
        self.angle = angle
        self.speed = speed
        self.qos_profile = qos_profile
        self._goal_handle = None
        self._goal_lock = threading.Lock()

        # Initialize node
        super().__init__(NODE_NAME.format(name=self.name))

        # Initialize topics
        self.angle_topic_publisher = self.create_publisher(
            Float64, ANGLE_TOPIC_NAME.format(name=self.name), qos_profile
        )
        self.speed_topic_publisher = self.create_publisher(
            Float64, SPEED_TOPIC_NAME.format(name=self.name), qos_profile
        )

        # Initialize services
        self.speed_service = self.create_service(
            Speed,
            SPEED_SERVICE_NAME.format(name=self.name),
            self.speed_service_callback,
        )
        self.initialize_service = self.create_service(
            Initialization,
            INITIALIZE_SERVICE_NAME.format(name=self.name),
            self.initialize_service_callback,
        )

        # Initialize actions
        self.angle_action_server = ActionServer(
            self,
            Angle,
            ANGLE_ACTION_NAME.format(name=self.name),
            execute_callback=self.angle_execute_callback,
            goal_callback=self.angle_goal_callback,
            handle_accepted_callback=self.angle_handle_accepted_callback,
            cancel_callback=self.angle_cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

        # Bounds
        self.angle_lower_bound = angle_lower_bound
        self.angle_upper_bound = angle_upper_bound
        self.speed_lower_bound = speed_lower_bound
        self.speed_upper_bound = speed_upper_bound

        # Initialize topics
        time.sleep(1)
        self.initialize_topic_values()

    def angle_goal_callback(self, goal_request):
            """Accept or reject a client request to begin an action."""
            # Check if valid request
            self.get_logger().info('Received goal request')
            return GoalResponse.ACCEPT

    def angle_handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        goal_handle.execute()

    def angle_cancel_callback(self, goal):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def angle_execute_callback(self, goal_handle):
        """Provides responses and feedback for angle adjustment actions"""
        self.get_logger().info(
            f"[Action] Received angle adjustment request = {str(self.angle)} -> {str(goal_handle.request.requested_angle)}"
        )
        if (
            goal_handle.request.requested_angle >= self.angle_lower_bound
            and goal_handle.request.requested_angle <= self.angle_upper_bound
        ):  # In valid range
            self.get_logger().info(
                f"[Action] Accepted angle adjustment request = {str(goal_handle.request.requested_angle)}"
            )
            ITERATIONS = 10
            delta = (goal_handle.request.requested_angle - self.angle) / ITERATIONS
            feedback_msg = Angle.Feedback()
            feedback_msg.current_angle = self.angle

            while round(self.angle) != round(
                goal_handle.request.requested_angle, 0
            ):  # Change
                # Publish feedback
                feedback_msg.current_angle = self.angle
                goal_handle.publish_feedback(feedback_msg)

                # Publish angle update
                self.update_angle()
                self.get_logger().info(f"[Action] Current angle = {self.angle}")
                self.angle += delta
                time.sleep(1)

            goal_handle.succeed()
        else:
            self.get_logger().info(
                f"[Service] Rejected angle adjustment request = {str(goal_handle.request.requested_angle)}"
            )
            goal_handle.abort()

        # Publish angle update
        self.update_angle()

        # Publish result
        result = Angle.Result()
        result.final_angle = self.angle

        return result

    def initialize_topic_values(self):
        angle_msg = Float64()
        speed_msg = Float64()
        angle_msg.data = self.angle
        speed_msg.data = self.speed
        self.angle_topic_publisher.publish(angle_msg)
        self.speed_topic_publisher.publish(speed_msg)
        self.get_logger().info(f"[Topic] Published angle = {str(angle_msg.data)}")
        self.get_logger().info(f"[Topic] Published speed = {str(speed_msg.data)}")

    def speed_service_callback(self, request, response):
        """Provides responses for speed adjustment services"""
        self.get_logger().info(
            f"[Service] Received speed adjustment request = {str(self.speed)} -> {str(request.speed)}"
        )
        if (
            request.speed >= self.speed_lower_bound
            and request.speed <= self.speed_upper_bound
        ):  # In valid range
            self.speed = request.speed
            response.response = True
            self.get_logger().info(
                f"[Service] Accepted speed adjustment request = {str(request.speed)}"
            )
        else:  # Not in valid range
            response.response = False
            self.get_logger().info(
                f"[Service] Rejected speed adjustment request = {str(request.speed)}"
            )
        return response
    
    def initialize_service_callback(self, request, response):
        """Provides responses for initialize services"""
        self.get_logger().info(f"[Service] Received initialize request")

        with self._goal_lock:
        # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('[Service] Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
            self.get_logger().info("[Service] Accepted intialize request")
            self.angle = 0.0
            response.response = True
        return response

    def update_angle(self):
        angle_msg = Float64()
        angle_msg.data = self.angle
        self.angle_topic_publisher.publish(angle_msg)
