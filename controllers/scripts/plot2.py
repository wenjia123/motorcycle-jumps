from numpy import loadtxt
import matplotlib.pyplot as plt
import numpy as np

# Load data
data = loadtxt('jump2_closeloop_suspension_data_PD.txt', delimiter=',')
data1 = loadtxt('jump2_closeloop_suspension_data_PI.txt', delimiter=',')
data2 = loadtxt('openloop2__suspension_webots_data.txt', delimiter=',')
data3 = loadtxt('jump2_closeloop_suspension_data_speedonly.txt', delimiter=',')
data4 = loadtxt('jump2_closeloop_suspension_data_takeoffonly.txt', delimiter=',')

# Extract time and pitch angle
t  = data[:,0]
p  = data[:,2]
suspension_com = data[:,5]

t1 = data1[:,0]
p1 = data1[:,2]
suspension_com1 = data1[:,5]

t2 = data2[:,0]
p2 = data2[:,2]
suspension_com2 = data2[:,4]


t3 = data3[:,0]
p3 = data3[:,2]
suspension_com3 = data3[:,5]


t4 = data4[:,0]
p4 = data4[:,2]
suspension_com4 = data4[:,5]


#Define the reference angles
theta_takeoff = 0.7
theta_landing = -0.26

# Function to find time closest to a given pitch angle threshold
def find_time_for_angle(time_array, pitch_array, angle):
    # Find index where pitch is closest to the given angle
    idx = np.argmin(np.abs(pitch_array - angle))
    return time_array[idx]

# Find the times at which each controller hits takeoff and landing angles
takeoff_time  = find_time_for_angle(t, p, theta_takeoff)
landing_time  = find_time_for_angle(t, p, theta_landing)

takeoff_time1 = find_time_for_angle(t1, p1, theta_takeoff)
landing_time1 = find_time_for_angle(t1, p1, theta_landing)

takeoff_time2 = find_time_for_angle(t2, p2, theta_takeoff)
landing_time2 = find_time_for_angle(t2, p2, theta_landing)

takeoff_time3 = find_time_for_angle(t3, p3, theta_takeoff)
landing_time3 = find_time_for_angle(t3, p3, theta_landing)

takeoff_time4 = find_time_for_angle(t4, p4, theta_takeoff)
landing_time4 = find_time_for_angle(t4, p4, theta_landing)

# Plot all pitch angles on the same figure
# plt.figure(figsize=(10,6))
# plt.plot(t, suspension_com,   label='Pitchrate PD Control', color='b')
# plt.plot(t1, suspension_com1, label='Pitchangle PI Control', color='r')
# plt.plot(t2, suspension_com2, label='Open-loop', color='g')
# plt.plot(t3, suspension_com3, label='Speed Only Control', color='m')
# plt.plot(t4, suspension_com4, label='Takeoff Only Control', color='c')

plt.figure(figsize=(10,6))
plt.plot(t, p,   label='Pitchrate PI Control', color='b')
plt.plot(t1, p1, label='Pitchangle PD Control', color='r')
plt.plot(t2, p2, label='Open-loop', color='g')
plt.plot(t3, p3, label='Speed Only Control', color='m')
plt.plot(t4, p4, label='Takeoff Only Control', color='c')

# Draw horizontal lines for takeoff and landing angles
plt.axhline(y=theta_takeoff, color='red', linestyle='--', label='Takeoff angle = 0.5 rad')
plt.axhline(y=theta_landing, color='orange', linestyle='--', label='Landing angle = 0.349 rad')

# Draw vertical lines for each dataset at takeoff and landing times
# Adjust colors/patterns as desired
plt.axvline(x=landing_time, color='b', linestyle='-.')

plt.axvline(x=landing_time1, color='r', linestyle='-.')

plt.axvline(x=landing_time2, color='g', linestyle='-.')

plt.axvline(x=landing_time3, color='m', linestyle='-.')

plt.axvline(x=landing_time4, color='c', linestyle='-.')

# plt.xlabel('Time (s)')
# plt.ylabel('suspension Comprasion(m)')
# plt.title('Comparison of Suspension compression(D=10m,R=5m,theta_t = 0.7 radians,theta_l = 0.26 radians)')
# plt.legend(loc='upper right')  # Move legend outside the plot if it's too crowded
# plt.tight_layout()
# plt.savefig('Suspension_compression2.png')
# plt.show()

plt.xlabel('Time (s)')
plt.ylabel('Pitch Angle (radians)')
plt.title('Comparison of Pitch Angles(D=10m,R=5m,theta_t = 0.7 radians,theta_l = 0.26 radians)')
plt.legend(loc='lower left')  # Move legend outside the plot if it's too crowded
plt.tight_layout()
plt.savefig('Pitch_angles_comparison2.png')
plt.show()
