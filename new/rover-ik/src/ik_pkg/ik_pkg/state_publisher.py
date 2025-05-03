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
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import pygame as pyg
import time
from scipy.spatial.transform import Rotation as R

#####################
# Variables
#####################
#last_run_time = time.time() #Can be used for timer, not in use
started = False


#Math
degree = pi / 180.0

#Joint States
axis0: float = 0.
axis1: float = 0.
axis2: float = 0.
axis3: float = 0.
wrist: float = 0.
continuous: float = 0.
ccw = True

#IK Related
urdf_file_name = 'arm12.urdf'
ik_tolerance = 1e-3 #Tolerance determines if IK was successful (in meters)

#Angles Ax_0, Ax_1, Ax_2, Ax_3, Wrist, Continuous
start_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
last_angles = start_angles
ik_angles = start_angles

#current_position = [0.0, 0.0, 0.0] # Not currently used
target_position = [0.756, 0, 0.442] # [0.756, 0, 0.442] starting target position with correct orientation
target_orientation = []


#Animation
moving_up = True
step = 0.03  # Max movement increment







#####################
# Helper Functions
#####################


def update_orientation(fk_matrix):
    #Extract the 3x3 rotation matrix from the 4x4 Homogeneous Transformation Matrix as orientation
    global target_orientation
    target_orientation = fk_matrix[:3, :3]
    return 






class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        # Check if controller input is enabled
        self.declare_parameter('enable_controller', False)
        self.enable_controller = self.get_parameter('enable_controller').value
        self.get_logger().info(f"Controller Enabled: {self.enable_controller}")

        #Initialize the joint_state publisher
        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        # message declarations
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_footprint'
        joint_state = JointState()


        loop_rate = self.create_rate(15) #Update Rate

        #pass in all the global variables
        global started#, axis0, axis1, axis2, axis3, wrist
        global target_position, target_orientation, last_angles
        global ik_angles

        axis0 = 0.0
        axis1 = 0.0
        axis2 = 0.0
        axis3 = 0.0
        wrist = 0.0
        continuous = 0.0


        ######
        ##IK
        ######
        urdf = os.path.join(
            get_package_share_directory('ik_pkg'),
            urdf_file_name)
        joint_mask = [False, True, True, False, True, False, True, False, True, True, False]
        arm_chain = Chain.from_urdf_file(urdf, active_links_mask=joint_mask) # Load the robotic chain from URDF
        
        #print([link.name for link in arm_chain.links])
        #['Base link', 'Axis_0_Joint', 'Axis_1_Joint', 
        #'Axis_1_to_Segment_1', 'Axis_2_Joint', 
        # 'Axis_2_to_Segment_2', 'Axis_3_Joint', 
        # 'Axis_3_to_Segment_3', 'Wrist_Joint', 'Continuous_Joint', 'Axis_4_C_to_Effector']


        ######
        ##CONTROLLER
        ######
        if not started:
            controller_detected = False
            if self.enable_controller:
                pyg.init() # Initialize Pygame
                pyg.joystick.init() # Initialize the joystick module
                self.get_logger().info("Waiting for a controller to connect...")
                while not controller_detected:
                    pyg.event.pump()  # Process events to detect new controllers
                    joystick_count = pyg.joystick.get_count()

                    if joystick_count > 0:
                        # Get the first detected joystick
                        controller = pyg.joystick.Joystick(0)
                        controller.init()
                        self.get_logger().info(f"Controller Connected: {controller.get_name()}\n\n")
                        controller_detected = True
                    else:
                        time.sleep(1)  # Reduce CPU usage while waiting




        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                # update joint_states in the message
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = ['Axis_0_Joint', 'Axis_1_Joint', 'Axis_2_Joint', 'Axis_3_Joint', 'Continuous_Joint', 'Wrist_Joint']
                joint_state.position = [axis0, axis1, axis2, axis3, continuous, wrist]

                # update transform (position of the robot in the world)
                odom_trans.header.stamp = now.to_msg()
                odom_trans.transform.translation.x = 0.0
                odom_trans.transform.translation.y = 0.0
                odom_trans.transform.translation.z = 0.0


                #global last_run_time #Can be used for timer, currently not in use
                
    
                if not started:
                    started = True
                    update_orientation(arm_chain.forward_kinematics(start_angles))
                    

                if started:
                    axis0 = 0.0
                    axis1 = 0.0
                    axis2 = 0.0
                    axis3 = 0.0
                    continuous = 0.0
                    wrist = 0.0
                    self.joint_pub.publish(joint_state)
                    self.broadcaster.sendTransform(odom_trans)

                    # This will adjust as needed per iteration
                    loop_rate.sleep()
                    return
                    if self.enable_controller:
                        pyg.event.pump()  # Process events to detect new controllers

                        LS_X = round(controller.get_axis(0),2)*.5#left x-axis
                        LS_Y = round(controller.get_axis(1),2)*.5#left y-axis
                        RS_X = round(controller.get_axis(2),2)*.5#right x-axis
                        RS_Y = round(controller.get_axis(3),2)*.5#right y-axis 
                        RB = controller.get_button(10) #Right Bumper

                        #Update target position
                        last_pos = target_position
                        if RB:
                            if(RS_X > 0.3):
                                wrist += step * RS_X
                            elif(RS_X < -0.3):
                                wrist += step * RS_X
                            last_angles[6] = wrist
                            update_orientation(arm_chain.forward_kinematics(last_angles))
                        else:
                            if(LS_X > 0.1 or LS_X < -0.3):
                                target_position[0] += step * LS_X 
                            if(LS_Y > 0.1 or LS_Y < -0.3):
                                target_position[1] += (step * LS_Y)
                            if(RS_Y > 0.1 or RS_Y < -0.3):
                                target_position[2] += (step * RS_Y)
                        
                        
                        

                    #Solve IK
                    #if last_pos != target_position:
                    try:
                        ik_angles = arm_chain.inverse_kinematics(target_position, target_orientation, orientation_mode="all")
                        last_angles = ik_angles
                        #self.get_logger().info(f"IK DONE: output angles: {ik_angles}")
                    except Exception as e:
                        self.get_logger().info(f"IK Error: {e}")
                        #ik_angles = arm_chain.inverse_kinematics(target_position)

                    

                    #Compare target_position to FK result
                    fk_matrix = arm_chain.forward_kinematics(ik_angles)
                    fk_position = fk_matrix[:3, 3]
                    error = np.linalg.norm(target_position - fk_position)
                    if error > ik_tolerance:
                        self.get_logger().info("No VALID IK Solution")
                    else:
                        self.get_logger().info(f"IK Solution Found. Error: {error}")
                

                    axis0 = ik_angles[2]
                    axis1 = ik_angles[3]
                    axis2 = ik_angles[4]
                    axis3 = ik_angles[5]
                    wrist = ik_angles[6]

                    #####
                    #IK Animation    
                    #####
                    #        
                    #global moving_up
                    
                    # if target_position[0] < 0.5:
                    #     moving_up = True
                    # elif target_position[0] > 1.0:
                    #     moving_up = False

                    # if moving_up:
                    #     target_position[0] += 0.01
                    # else:
                    #     target_position[0] -= 0.01

                    #Manual Animation
                    # if axis1 > 90*degree:
                    #     moving_up = False
                    # elif axis1 < -90*degree:
                    #     moving_up = True

                    # if moving_up:
                    #     axis0 = 0.0
                    #     axis1 += 1*degree
                    #     axis2 = 0.0
                    #     axis3 = 0.0
                    #     wrist = 0.0
                    # else:
                    #     axis0 = 0.0
                    #     axis1 -= 1*degree
                    #     axis2 = 0.0
                    #     axis3 = 0.0
                    #     wrist = 0.0


                    #time.sleep(0.1)

                else:

                    pass
                

                # if time.time() - last_run_time >= 10: # Timer can be used for testing
                #     # Update the last run time
                #     last_run_time = time.time()



                # publish the joint states and transform
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
