from numpy import loadtxt
import matplotlib.pyplot as plt

# Load the data
data = loadtxt('jump_closeloop_webots_data_pitchrate.txt', delimiter=',')
data1 = loadtxt('jump_closeloop_webots_data_pitchangle.txt', delimiter=',')
data2 = loadtxt('openloop_webots_data.txt', delimiter=',')

t = data[:,0]
pitchrate = data[:,1]
pitchangle = data[:,2]
speed = data[:,3]
Torque = data[:,4]

t1 = data1[:,0]
pitchrate1 = data1[:,1]
pitchangle1 = data1[:,2]
speed1 = data1[:,3]
Torque1 = data1[:,4]

t2= data2[:,0]
pitchrate2 = data2[:,1]
pitchangle2 = data2[:,2]
speed2 = data2[:,3]

# Create a figure with two subplots, one in each row (i.e., one column)
fig, (ax1, ax2,ax3) = plt.subplots(nrows=2, ncols=2, figsize=(6, 8), sharex=True)

ax1 = ax[0,0]
ax2 = ax[0,1]
ax3 = ax[1,0] 

# First subplot
ax1.plot(t, pitchangle, color='b')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Pitch Angle (radians)')
ax1.set_title('Pitch Angle (pitchrate PI Control)')

# Second subplot
ax2.plot(t1, pitchangle1, color='r')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Pitch Angle (radians)')
ax2.set_title('Pitch Angle (pitchangle PD Control)')

ax3.plot(t2, pitchangle2, color='r')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Pitch Angle (radians)')
ax3.set_title('Pitch Angle for openloop')

# Adjust layout to prevent overlapping labels
plt.tight_layout()

# Save the figure to a file
plt.savefig('Pitch_angles_comparison.png')

# Display the figure (optional)
plt.show()
