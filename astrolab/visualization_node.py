# Libraries
import rclpy, math, sys
import rclpy.executors
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

# Constants
NODE_NAME = "visualization_node"
TOPIC_QOS_PROFILE = 10
JOINT_TOPIC_NAME = "joint_states"
JOINT_MESSAGE_TYPE = JointState
ANGLE_MSG_TYPE = Float64
ARM_ANGLE_TOPIC_NAME = "arm_angle_topic"
TABLE_ANGLE_TOPIC_NAME = "table_angle_topic"
INITIAL_ARM_ANGLE = 0.0
INITIAL_TABLE_ANGLE = 0.0
UPDATE_INTERVAL = 1/30
MODEL_PARENT = "odom"
MODEL_CHILD = "world"
JOINT_NAMES = ["arm_angle"]

class VisualizationNode(Node):
    """ Node for the visualization. """

    def __init__(self):
        # Initialize node
        super().__init__(NODE_NAME)
        
        # TF Connection
        self.transformation_broadcaster = TransformBroadcaster(self, qos=TOPIC_QOS_PROFILE)
        self.joint_state_publisher = self.create_publisher(JOINT_MESSAGE_TYPE, JOINT_TOPIC_NAME, TOPIC_QOS_PROFILE)
        
        # Topic subscriptions
        self.arm_angle_subscription = self.create_subscription(
            ANGLE_MSG_TYPE,
            ARM_ANGLE_TOPIC_NAME,
            self.arm_angle_listener_callback,
            TOPIC_QOS_PROFILE,
        )
        
        self.table_angle_subscription = self.create_subscription(
            ANGLE_MSG_TYPE,
            TABLE_ANGLE_TOPIC_NAME,
            self.table_angle_listener_callback,
            TOPIC_QOS_PROFILE,
        )
        
        # Initial configuration
        self.arm_angle = INITIAL_ARM_ANGLE
        self.table_angle = INITIAL_TABLE_ANGLE
        
        # Start update timer
        self.create_timer(UPDATE_INTERVAL, self.update_model)

        # prevent unused variable warning
        self.arm_angle_subscription
        self.table_angle_subscription

    ###################
    # Topic callbacks #
    ###################

    def arm_angle_listener_callback(self, msg):
        """ Callback for the arm angle topic. """
        self.get_logger().info(f"[Topic]: Read arm angle = {msg.data}")
        self.arm_angle = msg.data
        
    def table_angle_listener_callback(self, msg):
        """ Callback for the table angle topic. """
        self.get_logger().info(f"[Topic]: Read table angle = {msg.data}")
        self.table_angle = msg.data

    ##################
    # Help functions #
    ##################

    def update_model(self):
        """ Updates the joint states of the model. """
        # Generate transformation
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = MODEL_PARENT
        odom_trans.child_frame_id = MODEL_CHILD
        now = self.get_clock().now()
        odom_trans.header.stamp = now.to_msg()
        
        # Broadcast new transformation
        self.transformation_broadcaster.sendTransform(odom_trans)
        
        # Generate new joint state
        joint_state = JointState()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = JOINT_NAMES
        joint_state.position = [math.radians(-self.arm_angle)]
        
        # Publish joint state
        self.joint_state_publisher.publish(joint_state)

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
        node = VisualizationNode()
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
