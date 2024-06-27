import rclpy, math, os, json, time, threading
from rclpy.node import Node
from rclpy.task import Future
from rclpy.action.client import ClientGoalHandle
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
NODE_NAME = "user_interface_node"
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
INITIAL_ELEVATION = 0.0
INITIAL_AZIMUTH = 0.0
INITIAL_SPEED = 0.2
BASE_DIRECTORY = os.path.abspath(os.path.dirname(__file__))
CONFIGURATION_PATH = os.path.join(BASE_DIRECTORY, "data/configuration.json")


class user_interface_node(Node):
    """Class that provides a node for user interfaces."""

    def __init__(self):
        super().__init__(NODE_NAME)

        # Topics
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

        # Services
        self.arm_speed_client = self.create_client(Speed, ARM_SPEED_SERVICE_NAME)
        self.table_speed_client = self.create_client(Speed, TABLE_SPEED_SERVICE_NAME)
        self.arm_initialize_client = self.create_client(
            Initialization, ARM_INITIALIZE_SERVICE
        )
        self.table_initialization_client = self.create_client(
            Initialization, TABLE_INITIALIZE_SERVICE
        )

        # Actions
        self.elevation_action_client = ActionClient(self, Angle, ARM_ANGLE_ACTION_NAME)
        self.azimuth_action_client = ActionClient(self, Angle, TABLE_ANGLE_ACTION_NAME)

        # Current configuration
        self.elevation = INITIAL_ELEVATION
        self.azimuth = INITIAL_AZIMUTH
        self.speed = INITIAL_SPEED

        # prevent unused variable warning
        self.arm_angle_subscription
        self.arm_speed_subscription
        self.table_angle_subscription
        self.table_speed_subscription

    ###################
    # Topic callbacks #
    ###################

    def arm_angle_listener_callback(self, msg):
        self.get_logger().info(f"[Topic]: Read arm angle = {msg.data}")
        self.elevation = float(msg.data)

    def arm_speed_listener_callback(self, msg):
        self.get_logger().info(f"[Topic]: Read arm speed = {msg.data}")
        # TODO: Speed correction
        self.speed = msg.data

    def table_angle_listener_callback(self, msg):
        self.get_logger().info(f"[Topic]: Read table angle = {msg.data}")
        self.azimuth = msg.data

    def table_speed_listener_callback(self, msg):
        self.get_logger().info(f"[Topic]: Read table speed = {msg.data}")
        # TODO: Speed correction
        self.speed = msg.data

    ########################
    # Service goal senders #
    ########################

    def change_arm_speed(self, speed):
        while not self.arm_speed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("[Service] Waiting for arm_speed_service...")

        request = Speed.Request()
        request.speed = speed
        future = self.arm_speed_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"[Service] Changed arm speed to {speed}")

    def change_table_speed(self, speed):
        while not self.table_speed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("[Service] Waiting for table_speed_service...")

        request = Speed.Request()
        request.speed = speed
        future = self.table_speed_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"[Service] Changed table speed to {speed}")

    ###########
    # Actions #
    ###########

    # -> Arm angle

    def arm_angle_cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("[Action] Arm angle goal successfully canceled")
        else:
            self.get_logger().info("[Action] Arm angle goal failed to cancel")

    def arm_angle_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("[Action] Arm angle goal rejected")
        else:
            self.arm_angle_goal_handle = goal_handle

            self.get_logger().info("[Action] Arm angle goal accepted")

    def arm_angle_feedback_callback(self, feedback):
        self.get_logger().info(
            f"[Action] Received arm angle feedback = {feedback.feedback.current_angle}"
        )

    def arm_angle_send_goal(self, angle):
        goal_msg = Angle.Goal()
        goal_msg.requested_angle = angle

        self.elevation_action_client.wait_for_server()
        self.get_logger().info(f"[Action] Sending arm angle goal request = {angle}")

        self.send_arm_angle_goal_future = self.elevation_action_client.send_goal_async(
            goal_msg, feedback_callback=self.arm_angle_feedback_callback
        )

        self.send_arm_angle_goal_future.add_done_callback(
            self.arm_angle_goal_response_callback
        )

    # -> Table angle

    def table_angle_cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("[Action] Table angle goal successfully canceled")
        else:
            self.get_logger().info("[Action] Table angle goal failed to cancel")

    def table_angle_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("[Action] Table angle goal rejected")
        else:
            self.table_angle_goal_handle = goal_handle

            self.get_logger().info("[Action] Table angle goal accepted")

    def table_angle_feedback_callback(self, feedback):
        self.get_logger().info(
            f"[Action] Received table angle feedback = {feedback.feedback.current_angle}"
        )

    def table_angle_send_goal(self, angle):
        goal_msg = Angle.Goal()
        goal_msg.requested_angle = angle

        self.azimuth_action_client.wait_for_server()
        self.get_logger().info(f"[Action] Sending table angle goal request = {angle}")

        self.send_table_angle_goal_future = self.azimuth_action_client.send_goal_async(
            goal_msg, feedback_callback=self.table_angle_feedback_callback
        )

        self.send_table_angle_goal_future.add_done_callback(
            self.table_angle_goal_response_callback
        )

    ###########
    # Getters #
    ###########

    def get_elevation(self):
        return self.elevation

    def get_azimuth(self):
        return self.azimuth

    ######################
    # High-level options #
    ######################

    def move(self, elevation_one, azimuth_one, elevation_two, azimuth_two):
        self.arm_angle_send_goal(elevation_one)
        self.table_angle_send_goal(azimuth_one)
        threading.Thread(
            target=self.next_move,
            kwargs={
                "elevation_one": elevation_one,
                "elevation_two": elevation_two,
                "azimuth_one": azimuth_one,
                "azimuth_two": azimuth_two
            },
        ).start()

    def next_move(self, elevation_one, azimuth_one, elevation_two, azimuth_two):
        while self.elevation != elevation_one or self.azimuth != azimuth_one:
            time.sleep(5)
    
        self.arm_angle_send_goal(elevation_two)
        self.table_angle_send_goal(azimuth_two)

    def calibrate(self, elevation, azimuth):
        self.arm_angle_send_goal(elevation)
        self.table_angle_send_goal(azimuth)

    def simulate(self, latitude, longitude, day, time):
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

        # Make request
        self.arm_angle_send_goal(math.degrees(elevation))
        self.table_angle_send_goal(math.degrees(azimuth))

    def abort(self):
        self.get_logger().info(f"[Action] Aborting current arm angle goal")
        arm_angle_future = self.arm_angle_goal_handle.cancel_goal_async()
        arm_angle_future.add_done_callback(self.arm_angle_cancel_done)
        self.get_logger().info(f"[Action] Aborting current table angle goal")
        table_angle_future = self.table_angle_goal_handle.cancel_goal_async()
        table_angle_future.add_done_callback(self.table_angle_cancel_done)


def main(args=None):
    rclpy.init(args=args)

    node = user_interface_node()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
