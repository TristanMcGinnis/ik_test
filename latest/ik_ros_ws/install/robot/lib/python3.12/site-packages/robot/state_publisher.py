from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

import numpy as np
import time
import math


#ik libs
from spatialmath import SE3
import roboticstoolbox as rtb
from roboticstoolbox.robot.Robot import Robot
from roboticstoolbox import DHRobot, RevoluteDH

# Function to get the vector from joystick input
def get_input_vector(left_stick_x, left_stick_y, right_stick_y):
    """
    Takes joystick input values and returns a vector.
    
    - left_stick_x controls the X-axis.
    - left_stick_y controls the Z-axis.
    - right_stick_y controls the Y-axis.
    
    Args:
    - left_stick_x (float): Input from the left stick for left-right movement (-1 to +1).
    - left_stick_y (float): Input from the left stick for up-down movement (-1 to +1).
    - right_stick_y (float): Input from the right stick for up-down movement (-1 to +1).

    Returns:
    - tuple: A tuple (x, y, z) representing the vector.
    """
    x = left_stick_x  # X-axis input (left-right)
    z = -left_stick_y  # Z-axis input (up-down)
    y = -right_stick_y  # Y-axis input (in-out)
    
    return (x, y, z)

# Function to calculate the unit vector and magnitude of a vector
def calculate_unit_vector_and_magnitude(vector):
    """
    Calculates the unit vector and magnitude of a given vector.
    
    Args:
    - vector (tuple): A tuple (x, y, z) representing the vector.
    
    Returns:
    - tuple: A tuple (unit_vector, magnitude) where unit_vector is (ux, uy, uz)
             and magnitude is a float.
    """
    x, y, z = vector
    magnitude = math.sqrt(x**2 + y**2 + z**2)
    
    if magnitude == 0:
        # Avoid division by zero; return a zero vector if magnitude is zero.
        unit_vector = (0, 0, 0)
    else:
        unit_vector = (x / magnitude, y / magnitude, z / magnitude)
    
    return unit_vector, magnitude



class CustomArm(DHRobot):
    def __init__(self):
        # Define DH parameters for each joint based on your description
        
        super().__init__([
            RevoluteDH(alpha=0, a=0, d=0.084, qlim=(-3.14, 3.14)),  # Axis 0

            RevoluteDH(alpha=1.571, a=0.0, d=0.0, qlim=(1.571, 4.712)),  # Axis 1

            RevoluteDH(alpha=1.571, a=0.0, d=0.506, qlim=(3.14, 3.14)),  # X
            
            RevoluteDH(alpha=1.571, a=0.0, d=0.0, qlim=(0, 3.14)),  # Axis 2

            RevoluteDH(alpha=0, a=0.435, d=0.0, q=0, qlim=(-1.571, 1.571)), # Axis 3

            RevoluteDH(alpha=1.571, a=0.199, d=0.0, q=0, qlim=(-1.571, 1.571)), # Axis 4

            RevoluteDH(alpha=1.571, a=0.052, d=0.0, q=0, qlim=(1.571, 1.571)), # Y

            RevoluteDH(alpha=1.571, a=0.0, d=0.176, q=0, qlim=(0, 0)), # Z
        ], name="CustomArm")
        
        # Optional: Define a default configuration (e.g., zero angles)
        self.qz = np.zeros(8)  # Zero configuration
        self.qr = np.array([0, 3.14, 3.14, 1.571, 0, 0, 1.571, 0])  # Custom ready position
        self.qn = np.array([0, 0, 0, 0, 0, 0, 0, 0])  # current position
        
        self.addconfiguration("qz", self.qz)
        self.addconfiguration("qr", self.qr)


robot = CustomArm()


# Specify the path to your URDF file
#urdf_path = "/home/tristan/Downloads/arm06.urdf" 

# Load the robot model
#robot = rtb.models.URDF.Robot(urdf_path)


# Initialize a variable to track the last time the block was run
last_run_time = time.time()

moving_up = True




class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        degree = pi / 180.0
        loop_rate = self.create_rate(30)

        # robot state
        axis0: float = 0.
        axis1: float = 0.
        axis2: float = 0.
        axis3: float = 0.
        wristdif: float = 0.
        continuous: float = 0.
        ccw = True

        current_pos = [0.0, 0.0, 0.0]

        started = False

        # message declarations
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_link'
        joint_state = JointState()

        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                #joint_state.name = ['swivel', 'tilt', 'periscope']
                joint_state.name = ['base_link_to_axis_0', 'axis_0_to_axis_1', 'axis_1_to_axis_2', 'axis_2_to_axis_3', 'axis_3_to_axis_4', 'axis_4_to_axis_5']
                joint_state.position = [axis0, axis1, axis2, axis3, wristdif, continuous]

                # update transform
                odom_trans.header.stamp = now.to_msg()
                odom_trans.transform.translation.x = 0.0
                odom_trans.transform.translation.y = 0.0
                odom_trans.transform.translation.z = 0.0


                global last_run_time
    

                if not started:
                    started = not started
                    #T = SE3(1.0, 1.5, 0.5) * SE3.OA([0, 1, 0], [0, 0, -1])
                    #T = SE3(0.15, 0, 0)
                    #sol = robot.ikine_LM(T, q0=robot.qz)
                    axis0 = robot.qr[0]
                    axis1 = robot.qr[1]-3.14
                    axis2 = robot.qr[3]-1.571
                    axis3 = robot.qr[4]
                    wristdif = robot.qr[5]
                    continuous = 0.0
                    #use FK to determine the current position
                    # robot.qn = [axis0, axis1, axis2, axis3, wristdif, continuous]
                    # T = robot.fkine(robot.qn)
                    # print(T)
                    # sol = robot.ikine_LM(T, q0=robot.qz)
                    # axis0 = sol.q[0]
                    # axis1 = sol.q[1]
                    # axis2 = sol.q[2]
                    # axis3 = sol.q[3]
                    # wristdif = sol.q[4]
                    # continuous = sol.q[5]
                    # self.get_logger().info("The current state is : "+ str(T))
                




                if time.time() - last_run_time >= 1.25:
                    
                    #self.get_logger().info("The angles of each joints are : "+ str(joint_state.position))
                    

                    
                    # Update the last run time
                    last_run_time = time.time()



                # send the joint state and transform
                self.joint_pub.publish(joint_state)
                self.broadcaster.sendTransform(odom_trans)

                # This will adjust as needed per iteration
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass


def main():
    node = StatePublisher()

if __name__ == '__main__':
    main()
