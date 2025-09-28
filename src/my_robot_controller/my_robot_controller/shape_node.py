#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String


class ShapeNode(Node):

    def __init__(self):
        super().__init__('shape_node')
        self.publisher_ = self.create_publisher(Int32, 'shape', 10)

    def publish_shape(self, user_shape):
        msg = Int32()
        msg.data = user_shape
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published shape code: {user_shape}")


class QuitPublisher(Node):

    def __init__(self):
        super().__init__('quit_publisher')
        self.publisher_ = self.create_publisher(String, 'quit', 10)

    def publish_quit(self):
        msg = String()
        msg.data = "Quit"
        self.publisher_.publish(msg)
        self.get_logger().info("Published quit signal")


def main(args=None):
    rclpy.init(args=args)

    shape_node = ShapeNode()
    quit_publisher = QuitPublisher()

    try:
        while rclpy.ok():
            user_shape = input(
                "\nChoose a shape by selecting the number:\n"
                " 1. inverted heart\n"
                " 2. Spiral\n"
                " 3. Star\n"
                " 4. Stop\n"
                " (or press 's' to quit)\n> "
            ).strip()

            if user_shape == 's':
                quit_publisher.publish_quit()
                break

            if user_shape in ['1', '2', '3', '4']:
                shape_node.publish_shape(int(user_shape))
            else:
                print("\n❌ Select a valid number\n")

    except KeyboardInterrupt:
        pass
    finally:
        shape_node.destroy_node()
        quit_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
