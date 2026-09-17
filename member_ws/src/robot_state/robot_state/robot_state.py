import rclpy
from rclpy.executors import ExternalShutdownException

from .state_node import StateNode


def main(args=None):
    rclpy.init(args=args)
    node = StateNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
