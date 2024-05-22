import rclpy
from astrolab.component_node import ComponentNode

class TableNode(ComponentNode): # TODO: Incorporate into component_node
    """Class that provides a node for the table of the Physical Sunlight Simulator"""

    def __init__(self):
        super().__init__(name='table', angle=0.0, speed=1.0, qos_profile=10, topic_timer_period_sec=0.5,
                 angle_lower_bound=0.0, angle_upper_bound=360.0, speed_lower_bound=0.0, speed_upper_bound=100.0)
        
def main(args=None):
    rclpy.init(args=args)

    node = TableNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
