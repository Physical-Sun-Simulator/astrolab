import rclpy, sys
from astrolab.component_node import ComponentNode
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException


class TableNode(ComponentNode):  # TODO: Incorporate into component_node
    """Class that provides a node for the table of the Physical Sunlight Simulator"""

    def __init__(self):
        super().__init__(
            name="table",
            angle=0.0,
            speed=0.2,
            qos_profile=10,
            angle_lower_bound=0.0,
            angle_upper_bound=360.0,
            speed_lower_bound=0.0,
            speed_upper_bound=0.4,
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
