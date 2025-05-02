import rclpy
from rclpy.node import Node
from .load_amr_traffic_rules import get_amr_traffic_rules


class TrafficRulesNode(Node):
    def __init__(self):
        super().__init__("traffic_rules_node")
        self.get_logger().info("Traffic Rules Node started!")
        self.load_traffic_rules()

    def load_traffic_rules(self):
        try:
            traffic_rules = get_amr_traffic_rules()
            self.get_logger().info("Traffic rules loaded successfully!")
            print(f"Participant: {traffic_rules.participant()}")
            print(f"Location: {traffic_rules.location()}")
        except Exception as e:
            self.get_logger().error(f"Could not load AMR traffic rules: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TrafficRulesNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
