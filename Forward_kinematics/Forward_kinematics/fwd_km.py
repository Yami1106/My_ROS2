# Author: Ashish Sukumar

import rclpy
from rclpy.node import Node
import math 
from std_msgs.msg import Float32MultiArray
# importing the numpy library to utizlie np arrays as matrices for forward kinematics
import numpy as np 

#creating a function that returns the DH transform matrix and takes DH parameters as its input
def DH_transform(a,theta,d,alpha):
    # variables are created for storing the sin and cos values so that it becomes easier to refer to them in the future
    ctheta, stheta = math.cos(theta) , math.sin(theta)
    calpha, salpha  = math.cos(alpha) , math.sin(alpha)

    # the np array stores the values in a matrix format 
    Ai =np.array([
        [ctheta , -stheta*calpha , stheta*salpha , a*ctheta],
        [stheta , ctheta*calpha , -ctheta*salpha , a*stheta],
        [0 , salpha , calpha , d],
        [0 , 0 , 0 , 1]],dtype=float) 
    
    return Ai
    
class Forward_km(Node):
    # this class will contain the function calls required to find the pose of the end effector

    def __init__(self):
        # this is the class constructor, this is where the class and all the required variables will be initialized 
        super().__init__('Forward_km')

        #initializing the link lengths to be 1 
        self.l1,self.l2,self.l3 = 1,1,1

        #Creating the subscriber with the datatype,topic name, callback function and the queue size as parameters
        self.subs = self.create_subscription(Float32MultiArray,'/HW3',self.DH_callback,10)

        #the logger function is used to display the text on the terminal 
        self.get_logger().info(f"Publish [q1,q2,q3] values in rad to topic DH")
    
    def DH_callback(self,msg):
        # this callback function calculates the homogeneous matrices(A1,A2,A3) for all the 3 links and also calculates the transform matrix(T) which gives the pose of the end effector 
        if len(msg.data)<3:
            # 3 values have to be published to the topic, this loop checks if there are 3 values and if not it warns the user to input 3 values 
            self._logger.warn("Expected 3 values q1,q2,q3 in radians.") 
        
        # since the published data is in an array format it has to be unpacked and this can be done by referring to the index values of the msg that is acquired from the topic 
        q1 = msg.data[0]
        q2 = msg.data[1]
        q3 = msg.data[2]

        # storing the DH parameters as lists helps in referring to them easier
        a = [0, self.l2, self.l3]
        theta = [q1, q2, q3]
        d = [self.l1, 0, 0]
        alpha = [-0.5*(math.pi), 0, 0]

        # The function DH_transform is called so that homogeneous matrices for each of the pair of parameters is created
        A1 = DH_transform(a[0], theta[0], d[0], alpha[0])
        A2 = DH_transform(a[1], theta[1], d[1], alpha[1])
        A3 = DH_transform(a[2], theta[2], d[2], alpha[2])

        # Using these values the transform matrix can be calculated which gives the end-effector pose, '@' is used to perform the matrix multiplication 

        T = A1 @ A2 @ A3

        # the printoptions function is used to print the transform matrix 
        with np.printoptions(precision = 3, suppress = True):
            self.get_logger().info(f"\nT=\n{T}")


def main(args=None):
    rclpy.init(args=args)
    node = Forward_km()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


