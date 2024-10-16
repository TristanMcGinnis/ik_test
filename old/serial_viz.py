import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import serial
import threading
import time

# Define the rotation matrix around the z-axis
def Rz(theta):
    theta = np.radians(theta)  # Convert degrees to radians
    return np.array([
        [np.cos(theta), -np.sin(theta), 0, 0],
        [np.sin(theta), np.cos(theta), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

# Transformation matrix for each joint
def transform_matrix(a, alpha, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), 0, a],
        [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha), -d * np.sin(alpha)],
        [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha), d * np.cos(alpha)],
        [0, 0, 0, 1]
    ])

link_lengths = [468, 378, 97]  # Link lengths of the arm
joint_angles = [0, 0, 0]  # Initial joint angles
data_lock = threading.Lock()  # Lock for accessing joint angles

# Setup the matplotlib plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-600, 600])
ax.set_ylim([-600, 600])
ax.set_zlim([0, 600])

def update_arm():
    with data_lock:
        temp_angles = joint_angles[:]
    point = np.array([0, 0, 0, 1])
    points = [point[:3]]
    T = np.eye(4)
    for length, angle in zip(link_lengths, temp_angles):
        T = T @ Rz(angle) @ transform_matrix(length, 0, 0, 0)
        point = T @ np.array([0, 0, 0, 1])
        points.append(point[:3])
    
    ax.cla()
    ax.set_xlim([-600, 600])
    ax.set_ylim([-600, 600])
    ax.set_zlim([0, 600])
    points = np.array(points)
    ax.plot(points[:,0], points[:,1], points[:,2], "o-")
    plt.draw()

def read_serial():
    try:
        ser = serial.Serial('COM5', 9600, timeout=1)
    except serial.SerialException:
        print("Failed to connect on COM5")
        return

    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            try:
                angles = list(map(float, line.split(',')))
                if len(angles) == 3:
                    with data_lock:
                        global joint_angles
                        joint_angles = angles
            except ValueError:
                print("Received non-float data")

# Start the serial reading in a separate thread
thread = threading.Thread(target=read_serial)
thread.start()

# Update the plot in the main thread
try:
    while True:
        update_arm()
        plt.pause(0.05)  # Adjust the pause time as needed
except KeyboardInterrupt:
    print("Stopped by user.")
