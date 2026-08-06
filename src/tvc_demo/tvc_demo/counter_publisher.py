import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class CounterPublisher(Node):
    def __init__(self):
        super().__init__('counter_publisher')
        self.publisher_ = self.create_publisher(Int32, '/tvc_demo/counter', 10)
        self.count = 0
        self.timer = self.create_timer(0.5, self.tick)

    def tick(self):
        msg = Int32()
        msg.data = self.count
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')  # print the published message to the console
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = CounterPublisher()
    rclpy.spin(node) # Keep the node running until it is shut down
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
