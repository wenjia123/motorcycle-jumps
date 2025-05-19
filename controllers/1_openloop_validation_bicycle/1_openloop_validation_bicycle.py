"""basic_drive_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, InertialUnit
from numpy import *
from Rollover import Rollover
from whipple_vs_webots import makePlot
#from realtime_plotter import RealTimePlot

driveVelocity= 4.35#3.95#28.95

stepTime = 0 #seconds, time of step response
stepTorque = 0 #Nm, amount of torque to apply to handlebars

# create the Robot instance.
robot = Robot()
yawCorr = Rollover()

recordData = True

if recordData:
    # start a file we can use to collect data
    f = open('step_data.txt','w')
    f.write("# time, speed, Torque, roll, rollrate, steer, steerrate\r\n")
    #f.close()


# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
motor = robot.getDevice('drive_motor')
steer = robot.getDevice('steering_motor')
motor.setPosition(float('inf'))
steer.setPosition(float('inf'))
steer.setTorque(0)

imu = robot.getDevice('imu')
imu.enable(timestep)
gps = robot.getDevice('gps')
gps.enable(timestep)
gyro = robot.getDevice('gyro')
gyro.enable(timestep)
steergyro = robot.getDevice('steergyro')
steergyro.enable(timestep)

steersensor = robot.getDevice('steer_angle')
steersensor.enable(timestep)

simtime = 0.0
yawRate = 0
oldYaw = 0

rollInt = 0
inteYawRate = 0
oldRoll = 0

eLane = 0
eLaneInt = 0

steerangle = 0
oldsteer = 0
steerRate = 0

oldRoll,oldPitch,oldYaw = imu.getRollPitchYaw()

firstLoop = True

#set the simulation forward speed and calculate rear wheel omega

Rrw = 0.3
driveOmega = driveVelocity/Rrw


# Main loop:
# - perform simulation steps until Webots is stopping the controller
#while robot.step(timestep) != -1:
while robot.step(timestep) != -1 and simtime<=12:
    simtime+=timestep/1000.0
    if(firstLoop):
        oldRoll,oldPitch,oldYaw = imu.getRollPitchYaw()
        oldYaw = yawCorr.update(oldYaw)
        oldsteer = steersensor.getValue()
        firstLoop=False
    # Read the sensors:
    #get current fwd speed
    U = gps.getSpeed()
    xyz = gps.getValues()
    curr_y = -xyz[1]
    #get IMU values and process to get yaw, roll rates
    #read IMU value
    rpy = imu.getRollPitchYaw()
    gyros = gyro.getValues()
    rollInt += (timestep/1000.0)*rpy[0]
    yaw = rpy[2]
    yaw = yawCorr.update(yaw)
    yawRate = gyros[2]
    (yaw-oldYaw)/(timestep/1000.0)
    #print("yaw/old: "+str(yaw)+","+str(oldYaw))

    oldYaw = yaw
    roll = rpy[0]
    rollRate = gyros[0]#(roll-oldRoll)/(timestep/1000.0)
    oldRoll = roll

    steerangle = steersensor.getValue()
    steerRate = (steerangle-oldsteer)/(timestep/1000.0)
    oldsteer = steerangle


    # Enter here functions to send actuator commands, like:
    #motor.setAvailableTorque(10)
    motor.setVelocity(driveOmega)

   #roll control
    #if(simtime>stepTime):
    if(simtime<=12):
       steerTorque = stepTorque
    else:
       steerTorque = 0

    steer.setTorque(steerTorque)

    # time, goalRoll, Torque, speed, roll, rollrate, pitch, pitchrate, intE
    #if(recordData and simtime>stepTime):
    if(recordData and simtime<=12):
        f.write(str(simtime-stepTime)+","+str(U)+","+str(steerTorque)+","+str(roll)+","+str(steerangle)+","+str(rollRate)+","+str(steerRate)+"\r\n")
        
f.close()
makePlot()
    #with open("whipple_vs_webots.py") as file:
        #exec(file.read())
