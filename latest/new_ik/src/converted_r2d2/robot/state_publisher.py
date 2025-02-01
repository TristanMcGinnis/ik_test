#ROS Stuff
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

#Functions
import numpy as np
import time
import math
from math import sin, cos, pi

#Needed for path to urdf
import os
from ament_index_python.packages import get_package_share_directory

#ik libs
import ikpy.chain
import pygame as pyg
import time


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

        ######
        ##IK
        ######

        #URDF

        urdf_file_name = 'arm10.urdf'
        urdf = os.path.join(
            get_package_share_directory('robot'),
            urdf_file_name)

        my_chain = ikpy.chain.Chain.from_urdf_file(urdf) # Load the robotic chain from URDF

        # Initial position and movement parameters
        step = 0.05  # Max movement increment
        x_pos, y_pos, z_pos = 0.1, 0, 0.4  # Start at lower bound

        ######
        ##CONTROLLER
        ######
        LS_X = 0.0
        LS_Y = 0.0
        RS_X = 0.0
        RS_Y = 0.0

        pyg.init() # Initialize Pygame
        pyg.joystick.init() # Initialize the joystick module
        self.get_logger().info("Waiting for a controller to connect...")

        controller_detected = False

        while not controller_detected:
            pyg.event.pump()  # Process events to detect new controllers
            joystick_count = pyg.joystick.get_count()

            if joystick_count > 0:
                # Get the first detected joystick
                controller = pyg.joystick.Joystick(0)
                controller.init()
                self.get_logger().info(f"Controller Connected: {controller.get_name()}")
                controller_detected = True
            else:
                time.sleep(1)  # Reduce CPU usage while waiting

        # Define starting target position 
        target_position = [x_pos, y_pos, z_pos]
        target_orientation = [0,0,1]


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
                joint_state.name = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5']
                joint_state.position = [axis0, axis1, axis2, axis3, wristdif]

                # update transform
                odom_trans.header.stamp = now.to_msg()
                odom_trans.transform.translation.x = 0.0
                odom_trans.transform.translation.y = 0.0
                odom_trans.transform.translation.z = 0.0


                global last_run_time
    

                if not started:
                    #started = not started
                    pyg.event.pump()  # Process events to detect new controllers

                    updated = False
                
                    # Solve IK and plot
                    #my_chain.plot(my_chain.inverse_kinematics(target_position, target_orientation, orientation_mode="all"), ax, target=target_position)

                    # Get controller input
                    LS_X = round(controller.get_axis(0),2)#left x-axis
                    LS_Y = round(controller.get_axis(1),2)#left y-axis
                    RS_X = round(controller.get_axis(2),2)#right x-axis
                    RS_Y = round(controller.get_axis(3),2)#right y-axis 

                    # Update x position
                    if(LS_X > 0.05 or LS_X < -0.05):
                        updated = True
                        x_pos += step * LS_X 
                    if(LS_Y > 0.05 or LS_Y < -0.05):
                        updated = True
                        y_pos += (step * LS_Y)
                    if(RS_Y > 0.05 or RS_Y <- 0.05):
                        updated = True
                        z_pos += (step * RS_Y)

                    # Update Target Position
                    target_position = [x_pos, y_pos, z_pos]
                    if updated:
                        self.get_logger().info(f"Target Position: {target_position}")

                    #self.get_logger().info(f"Distance between FK results: {distance}")
                    # axis0 = q[0]
                    # axis1 = q[1]
                    # axis2 = q[2]
                    # axis3 = q[3]
                    # wristdif = q[4]



                    # axis0 = sol.q[0]
                    # axis1 = sol.q[1]
                    # axis2 = sol.q[2]
                    # axis3 = sol.q[3]
                    # wristdif = 0.0

                else:

                    pass
                




                if time.time() - last_run_time >= 10:
                    
                    #self.get_logger().info("The angles of each joints are : "+ str(joint_state.position))
                    #T = robot.fkine(robot.qn)

                    
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
