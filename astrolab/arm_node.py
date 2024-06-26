import rclpy, sys
from astrolab.component_node import ComponentNode
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException


class ArmNode(ComponentNode):  # TODO: Incorporate into component_node
    """Class that provides a node for the arm of the Physical Sunlight Simulator"""

    def __init__(self):
        super().__init__(
            name="arm",
            angle=0.0,
            speed=0.3,
            qos_profile=10,
            angle_lower_bound=0.0,
            angle_upper_bound=90.0,
            speed_lower_bound=0.0,
            speed_upper_bound=0.6,
        )

def main(args=None):
    rclpy.init(args=args)
    try:
        node = ArmNode()
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
