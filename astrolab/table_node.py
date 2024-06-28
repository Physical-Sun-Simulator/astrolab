import rclpy, sys
from astrolab.component_node import ComponentNode
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException

# Constants
NAME = 'table'
INITIAL_ANGLE = 0
INITIAL_SPEED = 0.2
QOS_PROFILE = 10
ANGLE_LOWER_BOUND = 0.0
ANGLE_UPPER_BOUND = 360.0
SPEED_LOWER_BOUND = 0.0
SPEED_UPPER_BOUND = 0.4

class TableNode(ComponentNode):
    """Class that provides a node for the table of the Physical Sunlight Simulator"""

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
    rclpy.init(args=args)
    try:
        node = TableNode()
        executor = MultiThreadedExecutor()
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)


if __name__ == "__main__":
    main()
