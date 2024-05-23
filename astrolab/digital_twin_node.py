import rclpy, math
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

class digital_twin_node(Node):
    """Class that provides a node for the digital twin"""

    def __init__(self):
        super().__init__('digital_twin_node')
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.broadcaster = TransformBroadcaster(self, qos=10)
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
        
        # prevent unused variable warning
        self.arm_angle_subscription 
        self.arm_speed_subscription
        self.table_angle_subscription 
        self.table_speed_subscription
        
    def arm_angle_listener_callback(self, msg):
        self.get_logger().info('arm_angle_listener_callback: "%s"' % msg.data)
        self.updateModel(msg.data)
        
    def arm_speed_listener_callback(self, msg):
        self.get_logger().info('arm_speed_listener_callback: "%s"' % msg.data)
        
    def table_angle_listener_callback(self, msg):
        self.get_logger().info('table_angle_listener_callback: "%s"' % msg.data)
        
    def table_speed_listener_callback(self, msg):
        self.get_logger().info('table_speed_listener_callback: "%s"' % msg.data)
        
    def updateModel(self, angle):
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'world'
        joint_state = JointState()

        # update joint_state
        now = self.get_clock().now()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = ['swivel', 'tilt', 'periscope']
        joint_state.position = [0., math.radians(-angle), 0.]

        # update transform
        odom_trans.header.stamp = now.to_msg()

        # send the joint state and transform
        self.joint_pub.publish(joint_state)
        self.broadcaster.sendTransform(odom_trans)

def main(args=None):
    rclpy.init(args=args)

    node = digital_twin_node()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()