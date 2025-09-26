# Author: Ashish Sukumar

import rclpy
from rclpy.node import Node
import math 
from std_msgs.msg import Float32MultiArray
#import the Pose library from std_msgs 
from geometry_msgs.msg import Pose
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


#creting a function to calculate all the inverse kinematics which will be called in the inv_callback
def inv_solve(xc,yc,zc,a2,a3,d):
# the parameters for this function will be derived from what the user publishes
    # create a list that will store the q1,q2,q3 values
    sols = []

    # q1 for a spherical wrist is tan_inverse(y/x) and if x and y are both 0 it returns infinite solutions hence the none flag is given for that case
    q1 = None if (xc == 0.0 and yc == 0.0) else math.atan2(yc, xc)

    r  = math.hypot(xc, yc)

    # we want the distance from the shoulder joint not from the world frame, that is why l1 is subtracted from zc
    pz = zc - d 

    c3 = (r*r + pz*pz - a2*a2 - a3*a3) / (2*a2*a3)

    # this condition is written to make sure no errors occur and makes sure c3 is between -1 and 1 always
    c3 = max(min(c3, 1.0), -1.0)
    s3p = math.sqrt(max(0.0, 1.0 - c3*c3))

    # since there is a square root 2 values are possible so a loop is used to check and append both the values
    for s3 in (s3p, -s3p):        
        q3 = math.atan2(s3, c3)
        q2 = math.atan2(pz, r) - math.atan2(a3*s3, a2 + a3*c3)
        sols.append([q1, q2, q3])
    return sols


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

        #Creating a second subscriber for inverse kinematics
        self.inv_sub = self.create_subscription(Pose,"/HW4",self.inv_callback,10)
        # logger function to display that the node is listening to the topic HW4
        self.get_logger().info(f"Publish the end effector values.(x,y,z)")
    
    def DH_callback(self,msg):
        # this callback function calculates the homogeneous matrices(A1,A2,A3) for all the 3 links and also calculates the transform matrix(T) which gives the pose of the end effector 
        if len(msg.data)<3:
            # 3 values have to be published to the topic, this loop checks if there are 3 values and if not it warns the user to input 3 values 
            self.get_logger.warn("Expected 3 values q1,q2,q3 in radians.") 
        
        # since the published data is in an array format it has to be unpacked and this can be done by referring to the index values of the msg that is acquired from the topic 
        q1 = msg.data[0]
        q2 = msg.data[1]
        q3 = msg.data[2]

        # storing the DH parameters as lists helps in referring to them easier
        a = [0, self.l2, self.l3]
        theta = [q1, q2, q3]
        d = [self.l1, 0, 0]
        alpha = [0.5*(math.pi), 0, 0]

        # The function DH_transform is called so that homogeneous matrices for each of the pair of parameters is created
        A1 = DH_transform(a[0], theta[0], d[0], alpha[0])
        A2 = DH_transform(a[1], theta[1], d[1], alpha[1])
        A3 = DH_transform(a[2], theta[2], d[2], alpha[2])

        # Using these values the transform matrix can be calculated which gives the end-effector pose, '@' is used to perform the matrix multiplication 

        T = A1 @ A2 @ A3

        # the printoptions function is used to print the transform matrix 
        with np.printoptions(precision = 3, suppress = True):
            self.get_logger().info(f"\nT=\n{T}")

    def inv_callback(self,msg):
        # the end effector positions are extracted from the message that the user publishes
        xc= float(msg.position.x)
        yc= float(msg.position.y)
        zc = float(msg.position.z)

        # the solutions for q1,q2,q3 are stored in a variable sol by calling the function inv_solve
        sol = inv_solve(xc,yc,zc,self.l2,self.l3,self.l1)

        q1, q2, q3 = sol[0]
        q3_alt = sol[1][2]

        if q1 is None:
            self.get_logger().warn("q1 has infinite solutions")

        else:
            self.get_logger().info(
                f"Inverse kinematics for (x={xc:.3f}, y={yc:.3f}, z={zc:.3f}):\n"
                f"  q1={q1:.3f}, q2={q2:.3f}, q3={q3:.3f} (alt q3={q3_alt:.3f})"
            )




def main(args=None):
    rclpy.init(args=args)
    node = Forward_km()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


