# By Aqiel Oostenbrug (Jun 29, 2024)

# Libraries
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
INITIAL_GOAL_HANDLE = None
INITIAL_GOAL_LOCK = threading.Lock()
DIGITAL_TWIN_ANGLE_ITERATIONS = 10
DIGITAL_TWIN_ANGLE_INTERVAL = 1

class MotorNode(Node):
    """ Node with common functionalities for motor components on the Physical Sunlight Simulator. """

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
        self.goal_handle = INITIAL_GOAL_HANDLE
        self.goal_lock = INITIAL_GOAL_LOCK

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

        # Initialize action
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

    ####################
    # Service handlers #
    ####################

    def speed_service_callback(self, request, response):
        """ Responds to speed adjustment service requests. """
        # Calculate new speed
        new_speed = request.speed * self.speed_upper_bound
        
        self.get_logger().info(
            f"[Service] Received speed adjustment request = {self.speed} -> {new_speed}"
        )
        
        if (
            new_speed >= self.speed_lower_bound
            and new_speed <= self.speed_upper_bound
        ):  # In valid range
            # Assign new speed
            self.speed = new_speed
            
            # Construct service response
            response.response = True
            
            self.get_logger().info(
                f"[Service] Accepted speed adjustment request = {new_speed}"
            )
            
            # Update speed topic
            speed_msg = Float64()
            speed_msg.data = request.speed
            
            self.angle_topic_publisher.publish(speed_msg)
        else:  # Not in valid range
            # Construct service response
            response.response = False
            
            self.get_logger().info(
                f"[Service] Rejected speed adjustment request = {new_speed}"
            )
            
        return response
    
    def initialize_service_callback(self, request, response):
        """ Provides responses for initialize services. """
        self.get_logger().info(f"[Service] Received initialize request")

        with self.goal_lock:
            # Only allow one goal at a time
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().info("[Service] Aborting previous goal")
                
                # Abort the existing goal
                self.goal_handle.abort()
            
            self.get_logger().info("[Service] Accepted intialize request")

            # TODO: Commence initialization procedure...
            
            # Assign new angle
            self.angle = 0.0
            
            # Publish angle update
            self.update_angle_topic()
            
            # Construct response
            response.response = True
        return response
    
    ###################
    # Action handlers #
    ###################
    
    def angle_goal_callback(self, goal_request):
            """ Callback for checking angle goal action requests. """
            self.get_logger().info("[Action] Received angle goal request")
            # TODO: Check goal request and provide fitting response
            return GoalResponse.ACCEPT

    def angle_handle_accepted_callback(self, goal_handle):
        """ Callback for accepting a single angle action goal handle. """
        with self.goal_lock:
            # Only allows one goal at a time
            if self.goal_handle is not None and self.goal_handle.is_active:
                self.get_logger().info("[Action] Aborting previous goal")
                
                # Abort the existing goal
                self.goal_handle.abort()
            # Assign new goal handle
            self.goal_handle = goal_handle

        # Commence angle adjustment
        goal_handle.execute()

    def angle_cancel_callback(self, goal):
        """ Callback for angle action cancel requests. """
        self.get_logger().info("[Action] Received cancel request")
        # TODO: Check goal request and provide fitting response
        return CancelResponse.ACCEPT

    def angle_execute_callback(self, goal_handle):
        """ Provides responses and feedback for angle adjustment actions. """
        self.get_logger().info(
            f"[Action] Received angle adjustment request = {self.angle} -> {goal_handle.request.requested_angle}"
        )
        if (
            goal_handle.request.requested_angle >= self.angle_lower_bound
            and goal_handle.request.requested_angle <= self.angle_upper_bound
        ):  # In valid range
            self.get_logger().info(
                f"[Action] Accepted angle adjustment request = {str(goal_handle.request.requested_angle)}"
            )
            # Start (digital twin) adjustment
            delta = (goal_handle.request.requested_angle - self.angle) / DIGITAL_TWIN_ANGLE_ITERATIONS
            feedback_msg = Angle.Feedback()

            while round(self.angle) != round(
                goal_handle.request.requested_angle, 0
            ):  # Change while approx. different
                
                # Check if goal is still active
                if not goal_handle.is_active:
                    self.get_logger().info(f"[Action] Angle adjustment goal got aborted = {str(goal_handle.request.requested_angle)}")
                    return Angle.Result()
                
                # Check if goal got requested to be canceled
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info(f"[Action] Angle adjustment goal canceled = {str(goal_handle.request.requested_angle)}")
                    return Angle.Result()
                
                # Publish angle updates
                feedback_msg.current_angle = self.angle

                goal_handle.publish_feedback(feedback_msg)
                self.update_angle_topic()
                self.get_logger().info(f"[Action] Current angle = {self.angle}")
                
                # Increment angle
                self.angle += delta
                time.sleep(DIGITAL_TWIN_ANGLE_INTERVAL)

            with self.goal_lock:
                # Check if goal is still active
                if not goal_handle.is_active:
                    self.get_logger().info(f"[Action] Angle adjustment goal got aborted = {str(goal_handle.request.requested_angle)}")
                    return Angle.Result()

            # Finish action request
            goal_handle.succeed()
        else:
            self.get_logger().info(
                f"[Action] Rejected angle adjustment request = {str(goal_handle.request.requested_angle)}"
            )
            
            # Finish action request
            goal_handle.abort()

        # Publish angle update
        self.update_angle_topic()

        # Format result
        result = Angle.Result()
        result.final_angle = self.angle

        return result

    ##################
    # Help functions #
    ##################

    def update_angle_topic(self):
        """ Update angle topic using object variable. """
        angle_msg = Float64()
        angle_msg.data = self.angle
        self.angle_topic_publisher.publish(angle_msg)
