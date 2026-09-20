import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

from .control_node import ArmControlNode


def main(args=None):
    rclpy.init(args=args)
    node = ArmControlNode()
    # Multi-threaded: an executing goal's control loop must not block the
    # joint subscription (which detects a fault while a goal is active) or a
    # newer goal's own accept/execute callbacks (which is how preemption
    # works). All three share one ReentrantCallbackGroup so they can run
    # concurrently.
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
