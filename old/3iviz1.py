import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def create_robot_arm(lengths, initial_angles, target_position):
    """
    Initialize the robotic arm simulation with given segment lengths, initial joint angles, and a target position.
    """
    arm_lengths = np.array(lengths, dtype=float)
    initial_joint_angles = np.array(initial_angles, dtype=float)

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
        joint_angles = np.copy(initial_joint_angles)
        learning_rate = 0.01
        tolerance = 1e-3
        max_iterations = 500
        history = []
        
        for iteration in range(max_iterations):
            positions = forward_kinematics(joint_angles)
            current_position = positions[-1]
            error = target_pos - current_position
            error_norm = np.linalg.norm(error)
            if error_norm < tolerance:
                print(f"Converged in {iteration} iterations.")
                break
            
            # Normalize error to have a constant step size
            error_direction = error / error_norm
            step_size = min(learning_rate, error_norm)  # Prevent overshooting when very close
            delta_angles = np.dot(np.linalg.pinv(jacobian(joint_angles)), error_direction) * step_size
            joint_angles += delta_angles
            history.append(positions)
        
        if error_norm >= tolerance:
            print("Did not converge to the target.")
        
        return joint_angles, history


    return inverse_kinematics

# Parameters setup
arm_lengths = [1.84, 1.49, .38]  # Lengths of each arm segment
initial_angles = [0.0, 0.0, 0.0]  # Initial angles are set to zero
target_position = [2, 1]  # Target position for the end effector

# Create the inverse kinematics function
inverse_kinematics = create_robot_arm(arm_lengths, initial_angles, target_position)
resulting_angles, positions_history = inverse_kinematics(target_position)

# Visualization setup
fig, ax = plt.subplots()
ax.set_xlim(-3, 3)
ax.set_ylim(-3, 3)
line, = ax.plot([], [], 'o-')
target_marker = ax.plot(target_position[0], target_position[1], 'ro')

def update(frame):
    line.set_data(frame[:, 0], frame[:, 1])
    return line,

# Extend the last frame to ensure it's visible longer
extended_history = positions_history + [positions_history[-1]] * 50

ani = FuncAnimation(fig, update, frames=extended_history, blit=True, interval=25, repeat=False)
plt.show()
