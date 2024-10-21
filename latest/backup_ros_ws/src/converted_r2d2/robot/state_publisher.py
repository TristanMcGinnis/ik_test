from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped


#IK-related libraries
import roboticstoolbox as rtb
from spatialmath import SE3

import numpy as np
import time
import math

import pybullet as p
import pybullet_data

global current_angles 
current_angles = [0, 0, 0, 0, 0, 0]

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




# Load the robot model from URDF
# Replace 'path_to_urdf' with the actual path to your robot's URDF file.
robot = rtb.ERobot.URDF("/home/tristan/Desktop/ik_test/latest/ik_ros_ws/src/converted_r2d2/urdf/arm05.urdf")



#remove the fixed joints from the active joints mask

# Initialize a variable to track the last time the block was run
last_run_time = time.time()

moving_up = True





# # Set all links to active by default
# active_links_mask = [True] * len(my_chain.links)

# # Manually set the indices of known fixed links to inactive (False)
# # For example, Axis3_to_WristBase is at index 7 (as indicated in the error)
# fixed_link_indices = [7, 10]  # Add the indices of all fixed links from the warnings
# for index in fixed_link_indices:
#     active_links_mask[index] = False


# # Apply the active_links_mask to the chain
# my_chain.active_links_mask = active_links_mask





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

        current_pos = [ 0.5, 0.5, 0.5]
        target_pos = [ 0.5, 0.5, 0.5]
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
                joint_state.name = ['base_link_to_Axis0', 'Axis0_to_Axis1', 'Seg1_to_Axis2', 'Seg2_to_Axis3', 'WristBase_to_WristDif', 'WristDif_to_Continuous']
                joint_state.position = [axis0, axis1, axis2, axis3, wristdif, continuous]

                # update transform
                odom_trans.header.stamp = now.to_msg()
                odom_trans.transform.translation.x = 0.0
                odom_trans.transform.translation.y = 0.0
                odom_trans.transform.translation.z = 0.0


                global last_run_time
    



                if not started:
                    started = not started
                    # Set up PyBullet simulation
                    p.connect(p.GUI)
                    p.setAdditionalSearchPath(pybullet_data.getDataPath())

                    # Load your URDF file into the simulation
                    robot_id = p.loadURDF('/home/tristan/Desktop/ik_test/latest/ik_ros_ws/src/converted_r2d2/urdf/arm05.urdf', useFixedBase=True)

                    # Define the end effector link index and target position
                    end_effector_link_index = 10  # Replace with the actual end effector link index
                    target_position = [0.2, 0.1, 0.3]

                    # Calculate the inverse kinematics solution
                    joint_angles = p.calculateInverseKinematics(
                        bodyUniqueId=robot_id,
                        endEffectorLinkIndex=end_effector_link_index,
                        targetPosition=target_position
                    )

                    print("Joint angles for reaching the target position:", joint_angles)
                    p.disconnect()
                    # # Current joint angles (radians), assumed to be a NumPy array
                    # current_joint_angles = np.array(current_angles)  # Replace with your arm's current angles

                    # # Define the desired end effector position as a transformation matrix
                    # # Replace (x, y, z) with your desired target position.
                    # desired_position = SE3(0.25, 0.25, 0.2)  # This is the target position in space

                    # # Solve for the inverse kinematics using the Damped Least Squares method
                    # # The `q0` parameter ensures that we start from the current joint angles for a small change.
                    # ik_solution = robot.ikine_GN(desired_position, q0=current_joint_angles)

                    # # Check if the solution was successful
                    # if ik_solution.success:
                    #     new_joint_angles = ik_solution.q
                    #     print("New joint angles to reach the target position:", new_joint_angles)
                    # else:
                    #     #print("IK solution not found.")
                    #     self.get_logger().error("IK solution not found")
                    # pass



                # if time.time() - last_run_time >= 1.25:
                #     # Current joint angles (radians), assumed to be a NumPy array
                #     current_joint_angles = np.array(current_angles)  # Replace with your arm's current angles

                #     # Define the desired end effector position as a transformation matrix
                #     # Replace (x, y, z) with your desired target position.
                #     desired_position = SE3(current_pos[0]+0.1, current_pos[1]+0.1, current_pos[2]+0.1)  # This is the target position in space

                #     # Solve for the inverse kinematics using the Damped Least Squares method
                #     # The `q0` parameter ensures that we start from the current joint angles for a small change.
                #     ik_solution = robot.ikine_LMS(desired_position, q0=current_joint_angles)

                #     # Check if the solution was successful
                #     if ik_solution.success:
                #         new_joint_angles = ik_solution.q
                #         print("New joint angles to reach the target position:", new_joint_angles)
                #     else:
                #         print("IK solution not found.")

                    
                #     self.get_logger().info("The angles of each joints are : "+ str(joint_state.position))
                    
                #     # Update the last run time
                #     last_run_time = time.time()



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
