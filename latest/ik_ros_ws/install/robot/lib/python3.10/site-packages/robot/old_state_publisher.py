from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped


#IK-related libraries
import ikpy.chain
import numpy as np
import time

global my_chain

#remove the fixed joints from the active joints mask

# Initialize a variable to track the last time the block was run
last_run_time = time.time()

moving_up = True

global current_angles 
current_angles = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]



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
        #tilt = 0.
        #tinc = degree
        #swivel = 0.
        #angle = 0.
        #height = 0.
        #hinc = 0.005

        global my_chain
        my_chain = ikpy.chain.Chain.from_urdf_file("/home/tristan/Desktop/ik_test/latest/ik_ros_ws/src/converted_r2d2/urdf/arm05.urdf")
        my_chain.active_links_mask = [False, True, True, False, True, False, True, False, True, True, False]

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
                # (moving in a circle with radius=2)
                odom_trans.header.stamp = now.to_msg()
                odom_trans.transform.translation.x = 0.0
                odom_trans.transform.translation.y = 0.0
                odom_trans.transform.translation.z = 0.0
                #odom_trans.transform.rotation = \
                #    euler_to_quaternion(0, 0, angle + pi/2) # roll,pitch,yaw


                global last_run_time
                global moving_up
                global current_angles
    



                if not started:
                    started = True
                    target_joints = my_chain.inverse_kinematics(target_pos)
                    self.get_logger().info(str(my_chain.inverse_kinematics(target_pos)))

                    #joint_state.position = my_chain.inverse_kinematics(target_pos)
                    axis0 = target_joints[1]
                    axis1 = target_joints[2]
                    axis2 = target_joints[4]
                    axis3 = target_joints[6]
                    wristdif = target_joints[8]
                    self.get_logger().info("The angles of each joints are : "+ str(joint_state.position))



                if time.time() - last_run_time >= 0.5:
                        # Run the block
                        if moving_up:
                            target_pos[0] += 0.0254
                        else:
                            target_pos[0] -= 0.0254

                        if target_pos[0] >= 0.75 or target_pos[0] <= 0:
                            moving_up = not moving_up

                        

                        #target_joints = my_chain.inverse_kinematics(target_pos)
                        #calculate the new angles with inverse kinematics including the starting_nodes_angles of the current position
                        current_angles[1] = joint_state.position[0]
                        current_angles[2] = joint_state.position[1]
                        current_angles[4] = joint_state.position[2]
                        current_angles[6] = joint_state.position[3]
                        current_angles[8] = joint_state.position[4]
                        target_joints = my_chain.inverse_kinematics(target_pos, current_pos)
                        
                        self.get_logger().info(str(my_chain.inverse_kinematics(target_pos)))

                        #joint_state.position = my_chain.inverse_kinematics(target_pos)
                        axis0 = target_joints[1]
                        axis1 = target_joints[2]
                        axis2 = target_joints[4]
                        axis3 = target_joints[6]
                        wristdif = target_joints[8]
                        self.get_logger().info("The angles of each joints are : "+ str(joint_state.position))
                        
                        current_pos = target_pos

                        # Update the last run time
                        last_run_time = time.time()



                # send the joint state and transform
                self.joint_pub.publish(joint_state)
                self.broadcaster.sendTransform(odom_trans)

                # Create new robot state
                #tilt += tinc
                #if tilt < -0.5 or tilt > 0.0:
                #    tinc *= -1
                #height += hinc
                #if height > 0.2 or height < 0.0:
                #    hinc *= -1
                #swivel += degree*1
                #angle += (degree/4)*1

                #Create new robot state
                #axis0 += degree 

                # if axis0 >= 1.5:
                #     ccw = False
                # elif axis0 <= -1.5:
                #     ccw = True

                # if ccw:
                #     axis0 += .75*degree
                #     axis1 += .75*degree
                #     axis2 += .75*degree
                #     axis3 += .75*degree
                # else:
                #     axis0 -= .75*degree
                #     axis1 -= .75*degree
                #     axis2 -= .75*degree
                #     axis3 -= .75*degree

                #wristdif = 0
                #continuous = 0

                # This will adjust as needed per iteration
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    node = StatePublisher()

if __name__ == '__main__':
    main()
