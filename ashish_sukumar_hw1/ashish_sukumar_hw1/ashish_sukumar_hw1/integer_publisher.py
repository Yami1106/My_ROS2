import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64

class Integer_publisher(Node):
    def __init__(self):
        super().__init__('Integer_publisher')
        self.publisher_ = self.create_publisher(Int64,'HW1',5)
        self.i = 0
        inc_time= 1
        self.timer = self.create_timer(inc_time,self.int_callback)
        

    def int_callback(self):
        x=Int64() 
        x.data = self.i
        self.publisher_.publish(x)

        self.get_logger().info(str(x.data))
        
        self.i+=1

def main(args=None):
    rclpy.init(args=args)

    integer_publisher = Integer_publisher()

    rclpy.spin(integer_publisher)

    integer_publisher.destroy_node()

    rclpy.shutdown()


if __name__=='__main__':
    main()
