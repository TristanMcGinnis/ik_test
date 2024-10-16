import ikpy.chain
import numpy as np
import ikpy.utils.plot as plot_utils
import matplotlib.pyplot as plt

import time

my_chain = ikpy.chain.Chain.from_urdf_file("Arm04.urdf")

target_position = [ 0.5, -0.5, 0.1]

start_time = time.time()
print("The angles of each joints are : ", my_chain.inverse_kinematics(target_position))
print("Time taken: ", time.time() - start_time)

start_time = time.time()
print("The position of the end effector is : ", my_chain.forward_kinematics(my_chain.inverse_kinematics(target_position)))
real_frame = my_chain.forward_kinematics(my_chain.inverse_kinematics(target_position))
print("Computed position vector : %s, original position vector : %s" % (real_frame[:3, 3], target_position))
print("Time taken: ", time.time() - start_time)

# Optional: support for 3D plotting in the NB
# If there is a matplotlib error, uncomment the next line, and comment the line below it.
# %matplotlib inline
#%matplotlib widget

fig, ax = plot_utils.init_3d_figure()
my_chain.plot(my_chain.inverse_kinematics(target_position), ax, target=target_position)
plt.xlim(-1, 1)
plt.ylim(-1, 1)
plt.show()