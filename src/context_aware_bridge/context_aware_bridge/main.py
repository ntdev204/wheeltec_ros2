import rclpy
from context_aware_bridge.bridge_node import ContextAwareBridgeNode


def main(args=None):
    rclpy.init(args=args)
    node = ContextAwareBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
