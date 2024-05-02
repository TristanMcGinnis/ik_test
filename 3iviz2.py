import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def create_robot_arm(lengths, initial_angles):
    """
    Initialize the robotic arm with specified segment lengths and initial joint angles.
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
            angle = np.sum(joint_angles[:i + 1])
            for j in range(i, n):
                partial_x -= arm_lengths[j] * np.sin(angle)
                partial_y += arm_lengths[j] * np.cos(angle)
                angle += joint_angles[j + 1] if j + 1 < n else 0
            jac[0, i] = partial_x
            jac[1, i] = partial_y
    
        return jac

    def inverse_kinematics(target_pos, joint_angles):
        learning_rate = 0.01
        tolerance = 1e-3
        max_iterations = 500
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

    return inverse_kinematics, forward_kinematics

def interactive_loop():
    arm_lengths = [0.4678, 0.3779, 0.0967]
    initial_angles = [0.0, 0.0, 0.0]
    inverse_kinematics, forward_kinematics = create_robot_arm(arm_lengths, initial_angles)
    current_angles = np.array(initial_angles, dtype=float)

    fig, ax = plt.subplots()
    ax.set_xlim(-3, 3)
    ax.set_ylim(-3, 3)
    line, = ax.plot([], [], 'o-')

    while True:
        target_input = input("Enter new target position (x,y) or type 'exit' to quit: ")
        if target_input.lower() == 'exit':
            break
        
        try:
            target_position = np.array(eval(target_input), dtype=float)
            current_angles, positions_history = inverse_kinematics(target_position, current_angles)
            
            for positions in positions_history:
                line.set_data(positions[:, 0], positions[:, 1])
                plt.draw()
                plt.pause(0.01)

            plt.plot(target_position[0], target_position[1], 'ro')
        except Exception as e:
            print("Error:", e)
            continue

    plt.close()

interactive_loop()
