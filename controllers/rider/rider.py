"""Motorcycle lane change controller."""
from math import radians, pi
import numpy as np
from controller import Robot, Motor
# create the Robot instance.
robot = Robot()
# rider parameters:
l1 = 0.32  # THIGH_LENGTH
l2 = 0.5   # TORSO_LENGTH
l2_middle = l2 / 2
theta1_rest_deg = 41.34  # the rest knee angle
theta2_rest_deg = 78.69 
theta1_rest = np.radians(theta1_rest_deg)
theta2_rest = np.radians(theta2_rest_deg)

Px_rest = l1*np.cos(theta1_rest)+l2_middle*np.cos(theta1_rest+theta2_rest)
Py_rest = l1*np.sin(theta1_rest)+l2_middle*np.sin(theta1_rest+theta2_rest)

# desired position of P:
Px_target = Px_rest+0.1
Py_target = Py_rest
def inverseK(Px, Py, l1, l2):
    d = np.sqrt(Px**2 + Py**2)
    
    cos_theta2 = (d**2 - l1**2 - l2**2) / (2 * l1 * l2)
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)  
    theta2 = np.pi - np.arccos(cos_theta2)

    cos_angle = (l1**2 + d**2 - l2**2) / (2 * l1 * d)
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    theta1 = np.arctan2(Py, Px) - np.arccos(cos_angle)
    return theta1, theta2

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Get motor devices
hip_motor = robot.getDevice("HipMotor")
knee_motor = robot.getDevice("KneeMotor")
# Enable torque feedback
hip_motor.enableTorqueFeedback(timestep)
knee_motor.enableTorqueFeedback(timestep)

# Set motors to position control mode with PID

hip_motor.setPosition(theta2_rest)
knee_motor.setPosition(theta1_rest)
print(f"theta1rest (knee): {theta1_rest}, theta2_rest (hip): {theta2_rest}")



# Calculate inverse kinematics
theta1, theta2 = inverseK(Px_target, Py_target, l1, l2_middle)
print(f"theta1 (knee): {theta1}, theta2 (hip): {theta2}")

# Main loop:
while robot.step(timestep) != -1:
    # Set motor positions (convert to degrees if needed)
    knee_motor.setPosition(theta1)
    hip_motor.setPosition(theta2)

   

print("Controller finished")