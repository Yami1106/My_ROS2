import rclpy
from rclpy.node import Node
import math 

from std_msgs.msg import Float32MultiArray

class Angle_sub(Node):

    def __init__(self):
        super().__init__('angle_sub')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'HW2',
            self.angle_callback,
            10
        )
        self.get_logger().info('Publish roll, pitch, yaw in radians to HW2')
    
    def angle_callback(self,msg:Float32MultiArray):
        if len(msg.data)<3:
            self._logger.warn("Expected 3 values roll,pitch,yaw in radians.") 

        roll= msg.data[0]
        pitch = msg.data[1]
        yaw = msg.data[2]


        cro = math.cos(0.5*roll)
        sro = math.sin(0.5*roll)
        cpi = math.cos(0.5*pitch)
        spi = math.sin(0.5*pitch)
        cya = math.cos(0.5*yaw)
        sya = math.sin(0.5*yaw)

        x = sro*cpi*cya - cro*spi*sya
        y =cro*spi*cya + sro*cpi*sya
        z = cro*cpi*sya - sro*spi*cya
        w =cro*cpi*cya + sro*spi*sya

        self.get_logger().info(
            f'Input roll pitch yaq (rad): ({roll:.6f}, {pitch:.6f}, {yaw:.6f})  '
            f'quaternion (x,y,z,w): ({x:.6f}, {y:.6f}, {z:.6f}, {w:.6f})'
        )

def main(args=None):
    rclpy.init(args=args)
    node = Angle_sub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
