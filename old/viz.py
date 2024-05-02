import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

arm_lengths = np.array([1, 1, 1, 1, 1])  # Lengths of each segment

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
            partial_x -= arm_lengths[j] * np.sin(angle)  # derivative w.r.t. x
            partial_y += arm_lengths[j] * np.cos(angle)  # derivative w.r.t. y
        jac[0, i] = partial_x
        jac[1, i] = partial_y

    return jac

def inverse_kinematics(target_position, initial_joint_angles):
    joint_angles = np.array(initial_joint_angles)
    learning_rate = 0.01
    tolerance = 1e-3
    max_iterations = 100
    history = []
    
    for _ in range(max_iterations):
        positions = forward_kinematics(joint_angles)
        current_position = positions[-1]
        error = target_position - current_position
        if np.linalg.norm(error) < tolerance:
            break
        
        jac = jacobian(joint_angles)
        delta_angles = np.dot(np.linalg.pinv(jac), error) * learning_rate
        joint_angles += delta_angles
        history.append(positions)
    
    return joint_angles, history

# Setup the visualization
target_position = np.array([3, 2])
initial_angles = np.zeros(5)
resulting_angles, positions_history = inverse_kinematics(target_position, initial_angles)

fig, ax = plt.subplots()
ax.set_xlim(-5, 5)
ax.set_ylim(-5, 5)
line, = ax.plot([], [], 'o-')
target_marker = ax.plot(target_position[0], target_position[1], 'ro')

def update(frame):
    line.set_data(frame[:, 0], frame[:, 1])
    return line,

ani = FuncAnimation(fig, update, frames=positions_history, blit=True, interval=100)
plt.show()
