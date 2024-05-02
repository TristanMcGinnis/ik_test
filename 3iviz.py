import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def create_robot_arm(lengths, initial_angles, target_position):
    arm_lengths = np.array(lengths, dtype=float)  # Ensure arm lengths are float
    initial_joint_angles = np.array(initial_angles, dtype=float)  # Ensure angles are float

    def forward_kinematics(joint_angles):
        positions = np.zeros((len(joint_angles) + 1, 2))
        current_angle = 0
        
        for i, (angle, length) in enumerate(zip(joint_angles, arm_lengths)):
            current_angle += angle
            positions[i + 1, 0] = positions[i, 0] + np.cos(current_angle) * length
            positions[i + 1, 1] = positions[i, 1] + np.sin(current_angle) * length
        
        return positions

    def jacobian(joint_angles):
        n = len(joint_angles)
        jac = np.zeros((2, n))
        end_effector = forward_kinematics(joint_angles)[-1]
        
        for i in range(n):
            partial_x = 0
            partial_y = 0
            for j in range(i, n):
                angle = np.sum(joint_angles[:j+1])
                partial_x -= arm_lengths[j] * np.sin(angle)
                partial_y += arm_lengths[j] * np.cos(angle)
            jac[0, i] = partial_x
            jac[1, i] = partial_y
    
        return jac

    def inverse_kinematics(target_pos):
        joint_angles = np.copy(initial_joint_angles)  # Use a copy to avoid modifying the original
        learning_rate = 0.01
        tolerance = 1e-3
        max_iterations = 300
        history = []
        
        for _ in range(max_iterations):
            positions = forward_kinematics(joint_angles)
            current_position = positions[-1]
            error = target_pos - current_position
            if np.linalg.norm(error) < tolerance:
                break
            
            jac = jacobian(joint_angles)
            delta_angles = np.dot(np.linalg.pinv(jac), error) * learning_rate
            joint_angles += delta_angles
            history.append(positions)
        
        return joint_angles, history

    return inverse_kinematics

# Parameters setup
arm_lengths = [1.84, 1.49, .38]  # Length of each arm segment
initial_angles = [0.0, 0.0, 0.0]  # Ensure initial angles are float
target_position = [2, 1]  # Target end effector position

# Create robot arm and perform IK
inverse_kinematics = create_robot_arm(arm_lengths, initial_angles, target_position)
resulting_angles, positions_history = inverse_kinematics(target_position)

# Visualization using Matplotlib
fig, ax = plt.subplots()
ax.set_xlim(-3, 3)
ax.set_ylim(-3, 3)
line, = ax.plot([], [], 'o-')
target_marker = ax.plot(target_position[0], target_position[1], 'ro')

def update(frame):
    line.set_data(frame[:, 0], frame[:, 1])
    return line,

ani = FuncAnimation(fig, update, frames=positions_history, blit=True, interval=25)
plt.show()
