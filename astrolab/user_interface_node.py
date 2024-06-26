import rclpy, math, os, json
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float64
from simulation_interfaces.srv import Initialization, Speed  # type: ignore
from simulation_interfaces.action import Angle  # type: ignore
# from sun_position import (
#     get_sun_elevation_angle,
#     get_sun_azimuth_angle,
#     get_hour_angle,
#     get_true_solar_time,
#     get_equation_of_time,
#     get_sun_declination,
# )

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
            TOPIC_MSG_TYPE, ARM_ANGLE_TOPIC_NAME, self.arm_angle_listener_callback, TOPIC_QOS_PROFILE
        )
        self.arm_speed_subscription = self.create_subscription(
            TOPIC_MSG_TYPE, ARM_SPEED_TOPIC_NAME, self.arm_speed_listener_callback, TOPIC_QOS_PROFILE
        )
        self.table_angle_subscription = self.create_subscription(
            TOPIC_MSG_TYPE, TABLE_ANGLE_TOPIC_NAME, self.table_angle_listener_callback, TOPIC_QOS_PROFILE
        )
        self.table_speed_subscription = self.create_subscription(
            TOPIC_MSG_TYPE, TABLE_SPEED_TOPIC_NAME, self.table_speed_listener_callback, TOPIC_QOS_PROFILE
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

    def arm_angle_listener_callback(self, msg):
        self.get_logger().info(f"[Topic]: Read arm angle = {msg.data}")
        self.elevation = float(msg.data)
        # Get configuration
        # with open(CONFIGURATION_PATH, "r") as file:
        #     configuration = json.load(file)
                
        # # Serialize configuration
        # configuration['elevation'] = self.elevation
        
        # # Write to file
        # with open(CONFIGURATION_PATH, "w") as file:
        #     json.dump(configuration, file) 

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

    def arm_send_goal(self, angle):
        goal_msg = Angle.Goal()
        goal_msg.requested_angle = angle

        self.elevation_action_client.wait_for_server()

        return self.elevation_action_client.send_goal_async(goal_msg)

    def table_send_goal(self, angle):
        goal_msg = Angle.Goal()
        goal_msg.requested_angle = angle

        self.azimuth_action_client.wait_for_server()

        return self.azimuth_action_client.send_goal_async(goal_msg)

    def get_elevation(self):
        return self.elevation

    def get_azimuth(self):
        return self.azimuth

    def move(self, elevation_one, azimuth_one, elevation_two, azimuth_two, speed):
        pass

    def calibrate(self, elevation, azimuth):
        # Set goals
        elevation_goal_msg = Angle.Goal()
        azimuth_goal_msg = Angle.Goal()
        elevation_goal_msg.requested_angle = elevation
        azimuth_goal_msg.requested_angle = azimuth

        # Send goals
        self.elevation_action_client.wait_for_server()
        self.elevation_action_client.send_goal_async(elevation_goal_msg)
        self.azimuth_action_client.wait_for_server()
        self.azimuth_action_client.send_goal_async(azimuth_goal_msg)

        # TODO: Fix goal guarantees
        # TODO: Provide return values

    def simulate(self, latitude, longitude, day, time):
        # Calculate sun position
        # equation_of_time = get_equation_of_time(day)
        # true_solar_time = get_true_solar_time(time, longitude, equation_of_time)
        # hour_angle = get_hour_angle(true_solar_time)
        # sun_declination = get_sun_declination(day)
        # elevation = math.degrees(get_sun_elevation_angle(latitude, sun_declination, hour_angle))
        # azimuth = math.degrees(get_sun_azimuth_angle(
        #     elevation, latitude, sun_declination, true_solar_time
        # ))

        # # Make request
        # self.calibrate(elevation, azimuth)
        pass

    def abort(self):
        pass

def main(args=None):
    rclpy.init(args=args)

    node = user_interface_node()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
