import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class ik_arm:
    def __init__(self, lengths):
        self.arm_lengths = np.array(lengths, dtype=float)
    
    def forward_kinematics(self, joint_angles):
        positions = np.zeros((len(joint_angles) + 1, 2))
        current_angle = 0
        
        for i, (angle, length) in enumerate(zip(joint_angles, self.arm_lengths)):
            current_angle += angle
            positions[i + 1, 0] = positions[i, 0] + np.cos(current_angle) * length
            positions[i + 1, 1] = positions[i, 1] + np.sin(current_angle) * length
        
        return positions

    def jacobian(self, joint_angles):
        n = len(joint_angles)
        jac = np.zeros((2, n))
        end_effector = self.forward_kinematics(joint_angles)[-1]
        
        for i in range(n):
            partial_x = 0
            partial_y = 0
            angle = np.sum(joint_angles[:i + 1])
            for j in range(i, n):
                partial_x -= self.arm_lengths[j] * np.sin(angle)
                partial_y += self.arm_lengths[j] * np.cos(angle)
                angle += joint_angles[j + 1] if j + 1 < n else 0
            jac[0, i] = partial_x
            jac[1, i] = partial_y
    
        return jac

    def Calculate(self, current_joint_angles, target_position):
        joint_angles = np.copy(current_joint_angles)
        learning_rate = 0.01
        tolerance = 1e-3
        max_iterations = 500
        
        for _ in range(max_iterations):
            positions = self.forward_kinematics(joint_angles)
            current_position = positions[-1]
            error = target_position - current_position
            if np.linalg.norm(error) < tolerance:
                break
            
            jac = self.jacobian(joint_angles)
            delta_angles = np.dot(np.linalg.pinv(jac), error) * learning_rate
            joint_angles += delta_angles
        
        return joint_angles
    
    def visualize_positions(self, joint_angles1, joint_angles2):
        positions1 = self.forward_kinematics(joint_angles1)
        positions2 = self.forward_kinematics(joint_angles2)

        # Plotting positions for the first set of joint angles
        #plt.figure(1)
        #plt.plot(positions1[:, 0], positions1[:, 1], 'o-')
        #plt.title('Position 1')
        #plt.xlim([-3, 3])
        #plt.ylim([-3, 3])
        #plt.grid(True)

        # Plotting positions for the second set of joint angles
        
        plt.figure(2)
        plt.plot(positions2[:, 0], positions2[:, 1], 'o-')
        plt.title('Position 2')
        plt.xlim([-3, 3])
        plt.ylim([-3, 3])
        plt.grid(True)

        plt.show()

# Usage Example
#arm_lengths = [0.4678, 0.3779, 0.0967]
arm_lengths = [10.0, 8.0, 6.0]
robot_arm = ik_arm(arm_lengths)
joint_angles1 = [0.0, 0.0, 0.0]
#joint_angles2 = [np.pi / 4, np.pi / 4, -np.pi / 4]
#joint_angles2 = robot_arm.Calculate(joint_angles1, [0.5,0.5])


# Calculate new joint angles for the given target position
calculated_angles = robot_arm.Calculate(joint_angles1, [12.0,9.0])
joint_angles2 = calculated_angles
print("Calculated Joint Angles:", np.degrees(calculated_angles))


robot_arm.visualize_positions(joint_angles1, joint_angles2)


