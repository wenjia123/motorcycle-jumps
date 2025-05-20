"""Motorcycle lane change controller."""
from math import radians, pi
import numpy as np
from controller import Robot, Motor
# create the Robot instance.
robot = Robot()
# rider parameters:
l1 = np.sqrt(0.2**2+0.25**2) # THIGH_LENGTH
l2 = np.sqrt(0.05**2+0.25**2)   # TORSO_LENGTH
theta1_rest = np.arctan(0.25/0.2)  # the rest knee angle
theta2_rest = np.pi-theta1_rest-np.arctan(0.25/0.05)

Px_rest = l1*np.cos(theta1_rest)+l2*np.cos(theta1_rest+theta2_rest)
Py_rest = l1*np.sin(theta1_rest)+l2*np.sin(theta1_rest+theta2_rest)

# desired position of P:
Px_target = Px_rest
Py_target = Py_rest
def inverseK(Px, Py, l1, l2):
    
    d = np.sqrt(Px**2+Py**2)
    theta1=np.arctan(Py/Px)-np.arccos((-l2**2+l1**2+d**2)/(2*l1*d))
    theta2=3.14159-np.arccos((-d**2+l1**2+l2**2)/(2*l1*l2))
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
theta1, theta2 = inverseK(Px_target, Py_target, l1, l2)
print(f"theta1 (knee): {theta1}, theta2 (hip): {theta2}")

# Main loop:
while robot.step(timestep) != -1:
    # Set motor positions (convert to degrees if needed)
    knee_motor.setPosition(theta1-theta1_rest)
    hip_motor.setPosition(theta2-theta2_rest)

   

print("Controller finished")