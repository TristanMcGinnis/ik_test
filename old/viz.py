import numpy as np
import matplotlib.pyplot as plt
import serial
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D

# Serial port for reading the data
# Define the serial port and baud rate
port = "COM5"  # Update this value
baud_rate = 9600

# Open the serial port
ser = serial.Serial(port, baud_rate)



# Define the rotation matrix around the z-axis
def Rz(theta):
    # Convert degrees to radians
    theta = np.radians(theta)
    return np.array([
        [np.cos(theta), -np.sin(theta), 0, 0],
        [np.sin(theta), np.cos(theta), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

# Transformation matrix for each joint (assuming joint rotates around z-axis)
def transform_matrix(a, alpha, d, theta):
    # Convert degrees to radians for consistency
    theta = np.radians(theta)
    return np.array([
        [np.cos(theta), -np.sin(theta), 0, a],
        [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha), -d * np.sin(alpha)],
        [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha), d * np.cos(alpha)],
        [0, 0, 0, 1]
    ])

# Initial link lengths and joint angles
link_lengths = [468, 378, 97]  # Adjust these values as per your arm's dimensions
joint_angles = [0, 0, 0]  # All joints start at 0 angle (in degrees)

# Plotting the robotic arm
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
plt.subplots_adjust(left=0.25, bottom=0.25)

# Arm drawing function
def update_arm():
    # Starting point of the robotic arm
    point = np.array([0, 0, 0, 1])
    points = [point[:3]]
    
    # Apply transformations for each joint and link
    T = np.eye(4)
    for i, (length, angle) in enumerate(zip(link_lengths, joint_angles)):
        T = T @ Rz(angle) @ transform_matrix(length, 0, 0, 0)
        point = T @ np.array([0, 0, 0, 1])
        points.append(point[:3])
    
    # Clear previous lines
    ax.cla()
    ax.set_xlim([-600, 600])
    ax.set_ylim([-600, 600])
    ax.set_zlim([0, 600])
    
    # Draw the arm
    points = np.array(points)
    ax.plot(points[:,0], points[:,1], points[:,2], "o-")
    
    plt.draw()

# Sliders for controlling joint angles
axcolor = 'lightgoldenrodyellow'
ax_angle1 = plt.axes([0.25, 0.1, 0.65, 0.03], facecolor=axcolor)
ax_angle2 = plt.axes([0.25, 0.05, 0.65, 0.03], facecolor=axcolor)
ax_angle3 = plt.axes([0.25, 0.15, 0.65, 0.03], facecolor=axcolor)

s_angle1 = Slider(ax_angle1, 'Joint 1', -180, 180, valinit=0)
s_angle2 = Slider(ax_angle2, 'Joint 2', -180, 180, valinit=0)
s_angle3 = Slider(ax_angle3, 'Joint 3', -180, 180, valinit=0)

def update(val):
    #joint_angles[0] = s_angle1.val
    #joint_angles[1] = s_angle2.val
    #joint_angles[2] = s_angle3.val
    try:
        # Read the serial data
        data = ser.readline().decode("utf-8").strip()
        print(data)
        
        # Split the data into joint angles
        new_joint_angles = list(map(int, data.split(",")))
        joint_angles[0] = new_joint_angles[0]
        joint_angles[1] = new_joint_angles[1]
        joint_angles[2] = new_joint_angles[2]
    except:
        pass
    update_arm()

s_angle1.on_changed(update)
s_angle2.on_changed(update)
s_angle3.on_changed(update)

update_arm()
plt.show()


while True:
    try:
        # Read the serial data
        data = ser.readline().decode("utf-8").strip()
        print(data)
        
        # Split the data into joint angles
        joint_angles = list(map(int, data.split(",")))
        
        # Update the joint angles and redraw the arm
        update_arm()
    except KeyboardInterrupt:
        break
