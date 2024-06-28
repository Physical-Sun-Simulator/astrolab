import rclpy, sys
from astrolab.component_node import MotorNode
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException

# Constants
NAME = 'arm'
INITIAL_ANGLE = 0.0
INITIAL_SPEED = 0.3
QOS_PROFILE = 10
ANGLE_LOWER_BOUND = 0.0
ANGLE_UPPER_BOUND = 90.0
SPEED_LOWER_BOUND = 0.0
SPEED_UPPER_BOUND = 0.6

class ArmNode(MotorNode):
    """ Node for the arm of the Physical Sunlight Simulator. """

    def __init__(self):
        super().__init__(
            name=NAME,
            angle=INITIAL_ANGLE,
            speed=INITIAL_SPEED,
            qos_profile=QOS_PROFILE,
            angle_lower_bound=ANGLE_LOWER_BOUND,
            angle_upper_bound=ANGLE_UPPER_BOUND,
            speed_lower_bound=SPEED_LOWER_BOUND,
            speed_upper_bound=SPEED_UPPER_BOUND,
        )

def main(args=None):
    """ First function to be executed. """
    # Initialize rclpy
    rclpy.init(args=args)
    
    # Node control flow
    try:
        # Start node
        node = ArmNode()
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        # Ignore
        pass
    except ExternalShutdownException:
        # Graceful termination
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

if __name__ == "__main__":
    main()
