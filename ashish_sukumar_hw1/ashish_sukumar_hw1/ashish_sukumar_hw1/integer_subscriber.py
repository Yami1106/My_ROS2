import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64

class Integer_subscriber(Node):

    def __init__(self):
        super().__init__('Integer_subscriber')
        self.subscription = self.create_subscription(Int64,
        'HW1',
        self.intsub_callback,
        5)
        self.subscription

    def intsub_callback(self,x):
        i = x.data
        if i%2 ==0:
            self.get_logger().info(f"I received {i}.It is an even number")
        else:
            self.get_logger().info(f"I received {i}.It is an odd number")


def main(args=None):
    rclpy.init(args=args)

    integer_subscriber = Integer_subscriber()

    rclpy.spin(integer_subscriber)

    integer_subscriber.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()

