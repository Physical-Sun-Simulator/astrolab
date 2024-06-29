# Libraries
import rclpy, math, os, threading, sys, rclpy.executors
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float64
from simulation_interfaces.srv import Initialization, Speed  # type: ignore
from simulation_interfaces.action import Angle  # type: ignore
from sun_position import (
    get_sun_elevation_angle,
    get_sun_azimuth_angle,
    get_hour_angle,
    get_true_solar_time,
    get_equation_of_time,
    get_sun_declination,
)

# Constants
NODE_NAME = "{name}_node"
ARM_ANGLE_TOPIC_NAME = "arm_angle_topic"
ARM_SPEED_TOPIC_NAME = "arm_speed_topic"
TABLE_ANGLE_TOPIC_NAME = "table_angle_topic"
TABLE_SPEED_TOPIC_NAME = "table_speed_topic"
ARM_SPEED_SERVICE_NAME = "arm_speed_service"
TABLE_SPEED_SERVICE_NAME = "table_speed_service"
ARM_INITIALIZE_SERVICE = "arm_initialize_service"
TABLE_INITIALIZE_SERVICE = "table_initialize_service"
ARM_ANGLE_ACTION_NAME = "arm_angle_action"
TABLE_ANGLE_ACTION_NAME = "table_angle_action"
TOPIC_MSG_TYPE = Float64
TOPIC_QOS_PROFILE = 10
INITIAL_ARM_ANGLE = 0.0
INITIAL_TABLE_ANGLE = 0.0
INITIAL_ARM_SPEED = 0.5
INITIAL_TABLE_SPEED = 0.5
BASE_DIRECTORY = os.path.abspath(os.path.dirname(__file__))
CONFIGURATION_PATH = os.path.join(BASE_DIRECTORY, "data/configuration.json")
INITIALIZATION_MESSAGE = "Requesting reset"
BLOCKING_INTERVAL = 0.5

class UserInterfaceNode(Node):
    """ Node for user interfaces. """

    def __init__(self, name):
        # Object variables
        self.name = name
        
        # Initialize node
        super().__init__(NODE_NAME.format(name=self.name))

        # Topic subscriptions
        self.arm_angle_subscription = self.create_subscription(
            TOPIC_MSG_TYPE,
            ARM_ANGLE_TOPIC_NAME,
            self.arm_angle_listener_callback,
            TOPIC_QOS_PROFILE,
        )
        self.arm_speed_subscription = self.create_subscription(
            TOPIC_MSG_TYPE,
            ARM_SPEED_TOPIC_NAME,
            self.arm_speed_listener_callback,
            TOPIC_QOS_PROFILE,
        )
        self.table_angle_subscription = self.create_subscription(
            TOPIC_MSG_TYPE,
            TABLE_ANGLE_TOPIC_NAME,
            self.table_angle_listener_callback,
            TOPIC_QOS_PROFILE,
        )
        self.table_speed_subscription = self.create_subscription(
            TOPIC_MSG_TYPE,
            TABLE_SPEED_TOPIC_NAME,
            self.table_speed_listener_callback,
            TOPIC_QOS_PROFILE,
        )

        # Service clients
        self.arm_speed_client = self.create_client(Speed, ARM_SPEED_SERVICE_NAME)
        self.table_speed_client = self.create_client(Speed, TABLE_SPEED_SERVICE_NAME)
        self.arm_initialize_client = self.create_client(
            Initialization, ARM_INITIALIZE_SERVICE
        )
        self.table_initialize_client = self.create_client(
            Initialization, TABLE_INITIALIZE_SERVICE
        )

        # Actions clients
        self.arm_angle_action_client = ActionClient(self, Angle, ARM_ANGLE_ACTION_NAME)
        self.table_angle_action_client = ActionClient(self, Angle, TABLE_ANGLE_ACTION_NAME)

        # Current configuration
        self.arm_angle = INITIAL_ARM_ANGLE
        self.table_angle = INITIAL_TABLE_ANGLE
        self.arm_speed = INITIAL_ARM_SPEED
        self.table_speed = INITIAL_TABLE_SPEED
        
        # Initialize events
        self.arm_finish_event = threading.Event()
        self.table_finish_event = threading.Event()

        # prevent unused variable warning
        self.arm_angle_subscription
        self.arm_speed_subscription
        self.table_angle_subscription
        self.table_speed_subscription

    ###################
    # Topic callbacks #
    ###################

    def arm_angle_listener_callback(self, msg):
        """ Callback for the arm angle topic. """
        self.get_logger().info(f"[Topic]: Read arm angle = {msg.data}")
        self.arm_angle = msg.data

    def arm_speed_listener_callback(self, msg):
        """ Callback for the arm speed topic. """
        self.get_logger().info(f"[Topic]: Read arm speed = {msg.data}")
        self.arm_speed = msg.data

    def table_angle_listener_callback(self, msg):
        """ Callback for the table angle topic. """
        self.get_logger().info(f"[Topic]: Read table angle = {msg.data}")
        self.table_angle = msg.data

    def table_speed_listener_callback(self, msg):
        """ Callback for the arm angle topic. """
        self.get_logger().info(f"[Topic]: Read table speed = {msg.data}")
        self.table_speeed = msg.data

    ########################
    # Service goal senders #
    ########################

    def change_arm_speed(self, speed):
        """ Send request to change arm speed. """
        self.arm_speed_client.wait_for_service()

        request = Speed.Request()
        request.speed = speed
        self.arm_speed_client.call(request)
        
    def initialize_arm(self):
        """ Send request to initialize arm. """
        self.arm_initialize_client.wait_for_service()
    
        request = Initialization.Request()
        request.request = INITIALIZATION_MESSAGE
        self.arm_initialize_client.call(request)

    def change_table_speed(self, speed):
        """ Send request to change table speed. """
        self.table_speed_client.wait_for_service()
        
        request = Speed.Request()
        request.speed = speed
        self.table_speed_client.call(request)
        
    def initialize_table(self):
        """ Send request to initialize table. """
        self.table_initialize_client.wait_for_service()
    
        request = Initialization.Request()
        request.request = INITIALIZATION_MESSAGE
        self.table_initialize_client.call(request)

    ###########
    # Actions #
    ###########
    
    def change_arm_angle(self, angle):
        """ Asynchronously send an arm angle action request. """
        goal_msg = Angle.Goal()
        goal_msg.requested_angle = angle

        self.arm_angle_action_client.wait_for_server()
        self.get_logger().info(f"[Action] Sending arm angle goal request = {angle}")
        self.arm_finish_event.clear()
        self.send_arm_angle_goal_future = self.arm_angle_action_client.send_goal_async(
            goal_msg, feedback_callback=self.arm_angle_feedback_callback
        )
        self.send_arm_angle_goal_future.add_done_callback(
            self.arm_angle_goal_response_callback
        )

    def arm_angle_goal_response_callback(self, future):
        """ Goal confirmation callback for asynchronous arm angle action requests. """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("[Action] Arm angle goal rejected")
            self.arm_finish_event.set()
        else:
            self.get_logger().info("[Action] Arm angle goal accepted")
            self.arm_angle_goal_handle = goal_handle
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(self.get_arm_angle_result_callback)

    def arm_angle_feedback_callback(self, feedback):
        """ Feedback callback for asynchronous arm angle action requests. """
        self.get_logger().info(
            f"[Action] Received arm angle feedback = {feedback.feedback.current_angle}"
        )
        
    def get_arm_angle_result_callback(self, future):
        """ Finish callback for asynchronous arm angle action requests. """
        self.arm_finish_event.set()
        
    def arm_angle_cancel_done(self, future):
        """ Cancellation callback for asynchronous arm angle action requests. """
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("[Action] Arm angle goal successfully canceled")
            self.arm_finish_event.set()
        else:
            self.get_logger().info("[Action] Arm angle goal failed to cancel")

    def change_table_angle(self, angle):
        """ Asynchronously send an table angle action request. """
        goal_msg = Angle.Goal()
        goal_msg.requested_angle = angle

        self.table_angle_action_client.wait_for_server()
        self.get_logger().info(f"[Action] Sending table angle goal request = {angle}")
        self.send_table_angle_goal_future = self.table_angle_action_client.send_goal_async(
            goal_msg, feedback_callback=self.table_angle_feedback_callback
        )
        self.send_table_angle_goal_future.add_done_callback(
            self.table_angle_goal_response_callback
        )
    
    def table_angle_goal_response_callback(self, future):
        """ Goal confirmation callback for asynchronous table angle action requests. """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("[Action] Table angle goal rejected")
            self.table_finish_event.set()
        else:
            self.table_angle_goal_handle = goal_handle
            self.get_logger().info("[Action] Table angle goal accepted")
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(self.get_table_angle_result_callback)
            
    def table_angle_feedback_callback(self, feedback):
        """ Feedback callback for asynchronous table angle action requests. """
        self.get_logger().info(
            f"[Action] Received table angle feedback = {feedback.feedback.current_angle}"
        )
        
    def get_table_angle_result_callback(self, future):
        """ Finish callback for asynchronous table angle action requests. """
        self.table_finish_event.set()
    
    def table_angle_cancel_done(self, future):
        """ Cancellation callback for asynchronous table angle action requests. """
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("[Action] Table angle goal successfully canceled")
            self.table_finish_event.set()
        else:
            self.get_logger().info("[Action] Table angle goal failed to cancel")

    ###########
    # Getters #
    ###########

    def get_arm_angle(self):
        """ Return arm angle. """
        return self.arm_angle

    def get_table_angle(self):
        """ Return table angle. """
        return self.table_angle

    ##################
    # Help functions #
    ##################
    def change_angles(self, arm_angle, table_angle):
        """ Synchronously move the arm and table to the specified angles. """
        # Reset events
        self.arm_finish_event.clear()
        self.table_finish_event.clear()
        
        # Start async calls
        self.change_arm_angle(arm_angle)
        self.change_table_angle(table_angle)
        
        # Block till completion
        self.arm_finish_event.wait()
        self.table_finish_event.wait()
        
    def initialize(self):
        """ Synchronously move the arm and table to the start position. """
        # Initialize threads
        arm_thread = threading.Thread(target=self.initialize_arm)
        table_thread = threading.Thread(target=self.initialize_table)
        
        # Start threads
        arm_thread.start()
        table_thread.start()
        
        # Wait for both to finish
        arm_thread.join()
        table_thread.join()

    ######################
    # High-level options #
    ######################


    def move(self, arm_angle_one, table_angle_one, arm_angle_two, table_angle_two, speed):
        """ Move arm and table from position one to position two with the specified speed. """
        # Initialize arm and table
        self.initialize()
        
        # Move to position one
        self.change_angles(arm_angle_one, table_angle_one)
        
        # Change speed
        self.change_arm_speed(speed)
        self.change_table_speed(speed)
        
        # Move to position two
        self.change_angles(arm_angle_two, table_angle_two)
        
        # Reset speed
        self.change_arm_speed(INITIAL_ARM_SPEED)
        self.change_table_speed(INITIAL_TABLE_SPEED)

    def calibrate(self, arm_angle, table_angle):
        """ Move arm and table to the provided position. """
        # Initialize arm and table
        self.initialize()
        
        # Move to arm_angle and table_angle
        self.change_angles(arm_angle, table_angle)

    def simulate(self, latitude, longitude, day, time):
        """ Move the arm and table to the position specified by the latitude, longitude, day and time. """
        # Calculate sun position
        equation_of_time = get_equation_of_time(day)
        true_solar_time = get_true_solar_time(
            time, math.radians(longitude), equation_of_time
        )
        hour_angle = get_hour_angle(true_solar_time)
        sun_declination = get_sun_declination(day)
        elevation = get_sun_elevation_angle(
            math.radians(latitude), sun_declination, hour_angle
        )
        azimuth = get_sun_azimuth_angle(
            elevation, math.radians(latitude), sun_declination, true_solar_time
        )

        # Initialize arm and table
        self.initialize()
        
        # Move to arm_angle and table_angle
        self.change_angles(math.degrees(elevation), math.degrees(azimuth))

    def abort(self):
        """ Abort current arm and table action jobs. """
        self.get_logger().info(f"[Action] Aborting current arm angle goal")
        arm_angle_future = self.arm_angle_goal_handle.cancel_goal_async()
        arm_angle_future.add_done_callback(self.arm_angle_cancel_done)
        self.get_logger().info(f"[Action] Aborting current table angle goal")
        table_angle_future = self.table_angle_goal_handle.cancel_goal_async()
        table_angle_future.add_done_callback(self.table_angle_cancel_done)

#########
# Setup #
#########

def main():
    """ First function to be executed. """
    # Initialize rclpy
    rclpy.init(args=None)
    
    # Node control flow
    try:
        # Start node
        node = UserInterfaceNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ignore
        pass
    except rclpy.executors.ExternalShutdownException:
        # Graceful termination
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)


if __name__ == "__main__":
    main()
