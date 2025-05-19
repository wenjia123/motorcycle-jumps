"""Motorcycle lane change controller."""

from controller import Robot, Motor, InertialUnit
from numpy import *
from matplotlib.pyplot import *
import control
import control.matlab as cnt

sys.path.insert(0, '../Models')
from whipple_model import *
from Lane_Controller import *
from Rollover import Rollover
from realtime_plotter import RealTimePlot
from closeloop_bicyclemodel_vs_Webots import makePlot

#flag to show extra plots for debugging
showPlots = False
#flag to record data or not
recordData = True

# create the Robot instance.
robot = Robot()
#create an anti-rollover object for yaw angle to prevent 0-360 jumps
yawCorr = Rollover()

# SIMULATION SETUP
#what speed do we run at?
driveVelocity = 4
#what is the lane change step magnitude in meters?
step_mag = 4
#at what time should we perform the step?
step_time = 3
#names of the parameters in the model, in the correct order.
param_names = ['a ','b ','c','hrf','mrf','xff','zff','mff','Rfw','mfw','Rrw','mrw','Jyyf','Jyyr','lam']
#parameters of the DR250
#MC_params = array([.6888,1.45,0.115,0.5186,158.1,1.25,0.7347,10,0.356,10,0.33,13,0.6657066,0.6554166,1.1])
MC_params = array([.3,1.02,.08,.9,85,.9,.7,4,.35,3,.3,3,3*.35**2,3*.3**2,1.25])

##### do eigenvalue study for debugging if necessary ######
if(showPlots):
    vstudy,restudy,imstudy = plotEigStudy(MC_params,True)


# CONTROL PARAMETERS
lastControlTime = 0
dTcontrol = 0.005

Rlqr = .1#.001
Qlqr = eye(6)#/10.0
# Qlqr[5,5]=1.0

########## GET LQR GAINS ################
if(showPlots):
    fig,ax = subplots()
#use our model in MC_Model.py to get the appropriate controller
KLQR,sys = getLQRy(driveVelocity,Q=Qlqr,R=Rlqr,params=MC_params)
Klqr = ravel(KLQR)
print("Klqr: "+str(KLQR))
print(Klqr)
if showPlots:
    yout,tout = cnt.step(sys)
    yout*=step_mag
    plot(tout,yout[:,0],label='U='+str(lqrspeeds[k]))

if(showPlots):
    xlabel('Time (s)')
    ylabel('Roll (rad)')
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles, labels)
    show()

def clamp(val,min,max):
    assert min<max
    if(val<min):
        val=min
    elif val>max:
        val=max
    return val

#limits the drive motor torque (for realism)
def setDriveMotorTorque(self,motor,command,omega):
    #Assume that the torque to the wheel is 650 N-m max
    #for now, just allow max torque, and assume brake is same
    motorTorque = clamp(command,-650,650)
    #set motor force
    motor.setTorque(motorTorque)


if recordData:
    # start a file we can use to collect data
    f = open('closeloop_bicycle_webots_data.txt','w')
    f.write("#time,goalRoll,Torque,speed,roll,laneposition,yaw,latpos,rollrate,steer,steerrate,intE\r\n")

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# get the handles to the sensors and actuators on the robot
motor = robot.getDevice('drive_motor')
steer = robot.getDevice('steering_motor')
motor.setPosition(float('inf'))

imu = robot.getDevice('imu')
imu.enable(timestep)
gps = robot.getDevice('gps')
gps.enable(timestep)
gyro = robot.getDevice('gyro')
gyro.enable(timestep)

steersensor = robot.getDevice('steer_angle')
steersensor.enable(timestep)


####### INITIALIZE VALUES FOR STEERING CONTROL ########
T = 0

simtime = 0.0
yawRate = 0
oldYaw = 0

rollInt = 0
inteYawRate = 0
oldRoll = 0

steerangle = 0
oldsteer = 0
steerRate = 0
goalroll=0

oldRoll,oldPitch,oldYaw = imu.getRollPitchYaw()
#flag to indicate first loop through the sim
firstLoop = True
#set the simulation forward speed and calculate rear wheel omega
Rrw = MC_params[10]
driveOmega = driveVelocity/Rrw

# Main loop:
# - perform simulation steps until Webots is stopping the controller
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
    #get GPS reading
    xyz = gps.getValues()
    #get IMU values and process to get yaw, roll rates
    #read IMU value
    rpy = imu.getRollPitchYaw()
    gyros = gyro.getValues()
    yaw = rpy[2]
    yaw = yawCorr.update(yaw)
    yawRate = gyros[2]
    oldYaw = yaw
    roll = rpy[0]
    rollRate = gyros[0]#(roll-oldRoll)/(timestep/1000.0)
    #lateral position is just Y in this world
    latPos = xyz[1]

    #now get steer values and calculate steer rate.
    # WEBOTS is in ISO (z up) but Papa is in SAE (z down) so need to flip dir.
    steerangle = steersensor.getValue()
    steerRate = (steerangle-oldsteer)/(timestep/1000.0)
    oldsteer = steerangle

    ################### FINISH READING SENSORS, BEGIN  lane control ##########
    #now figure out the commanded rear wheel angular velocity
    driveOmega = driveVelocity/MC_params[10]
    #now set to that velocity (TODO make speed controller!)
    motor.setVelocity(driveOmega)

    #update goal Y as appropriate:
    stepVal = 0
    if(simtime>=step_time):
        stepVal = step_mag

    #do control only if enough time has passed
    if((simtime-lastControlTime)>dTcontrol):
        #now create errors from LQR
        goalYaw = 0#roadyaw + prev_y[0]/Sprev
        yawError = goalYaw - yaw
        # goalRoll = 0
        eRoll = 0 - roll
        eY = stepVal - latPos
        #goal steer based on goal yaw rate
        goalYawRate = 0

        # compute steer torque based on our control law.
        T = Klqr[0]*(eRoll) - Klqr[1]*steerangle - Klqr[2]*rollRate - Klqr[3]*steerRate + Klqr[4]*yawError + Klqr[5]*eY
        Tlim = 1000
        if(T>Tlim):
            T = Tlim
        elif(T<-Tlim):
            T = -Tlim

        # print("speed = "+str(U)+", Tq: "+str(T)+", prev error: "+str(prev_y[0])+", yaw error: "+str(yawError)+", yaw: "+str(yaw)+", goalYaw: "+str(goalYaw) )
        print("U: "+str(U)+", roll: "+str(roll)+", steer: "+str(steerangle)+", T="+str(T))
        # print("Torque: "+str(T))
        steer.setControlPID(0.0001,0,0)
        steer.setPosition(float('inf'))
        # WEBOTS is in ISO (z up) but Papa is in SAE (z down) so need to flip dir.
        steer.setTorque(T)
        lastControlTime = simtime
    if(recordData and simtime<=12):
        f.write(str(simtime)+","+str(goalroll)+","+str(T)+","+str(U)+","+str(roll)+","+str(stepVal)+","+str(yaw)+","+str(latPos)+","+str(rollRate)+","+str(steerangle)+","+str(steerRate)+"\r\n")
f.close()
makePlot()