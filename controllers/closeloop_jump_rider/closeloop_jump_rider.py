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

#from closeloop_motocyclemodel_vs_Webots import makePlot

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
driveVelocity =17#6.63#16.1#15.7#15.2697#7.6348
Udes = driveVelocity
#what is the lane change step magnitude in meters?
step_mag = 0
#at what time should we perform the step?
step_time = .5
hr = 0.635
#names of the parameters in the model, in the correct order.
param_names = ['a ','b ','c','hrf','mrf','xff','zff','mff','Rfw','mfw','Rrw','mrw','Jyyf','Jyyr','lam']
#parameters of the DR250
MC_params = array([.6888,1.45,0.115+hr,0.5186,88,1.25,0.7347,10,0.356,10,0.33,13,0.6657066,0.6554166,1.1])
mrw = MC_params[11]
mff = MC_params[7]
xff = MC_params[5]
mfw = MC_params[9]
Jyyr = MC_params[13]
Jyyf = MC_params[12]
##### do eigenvalue study for debugging if necessary ######
if(showPlots):
    vstudy,restudy,imstudy = plotEigStudy(MC_params,True)
a = MC_params[0]
b = MC_params[1]
h = MC_params[3]

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
#print("Klqr: "+str(KLQR))
#print(Klqr)
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
    f = open('../../scripts/closeloop_jump_pd/closeloop_pd_rider_backward0.1_webots_data.txtt','w')
    #f.write("#time,pitchRate,pitchangle,speed,Torqueclamped\r\n")

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())


l1 = np.sqrt(0.2**2+0.25**2) # THIGH_LENGTH
l2 = np.sqrt(0.05**2+0.25**2)   # TORSO_LENGTH
theta1_rest = np.arctan(0.25/0.2)  # the rest knee angle
theta2_rest = np.pi-theta1_rest-np.arctan(0.25/0.05)
print(l1,l2)
Px_rest = l1*np.cos(theta1_rest)+l2*np.cos(theta1_rest+theta2_rest)
Py_rest = l1*np.sin(theta1_rest)+l2*np.sin(theta1_rest+theta2_rest)

# desired position of P:
Px_target = Px_rest+0.1
Py_target = Py_rest
def inverseK(Px, Py, l1, l2):
    
    d = np.sqrt(Px**2+Py**2)
    theta1=np.arctan(Py/Px)-np.arccos((-l2**2+l1**2+d**2)/(2*l1*d))
    theta2=3.14159-np.arccos((-d**2+l1**2+l2**2)/(2*l1*l2))
    return theta1, theta2
    
# Get motor devices
hip_motor = robot.getDevice("HipMotor")
knee_motor = robot.getDevice("KneeMotor")
# Enable torque feedback
hip_motor.enableTorqueFeedback(timestep)
knee_motor.enableTorqueFeedback(timestep)

# Set motors to position control mode with PID

hip_motor.setPosition(theta2_rest)
knee_motor.setPosition(theta1_rest)

# Calculate inverse kinematics
theta1, theta2 = inverseK(Px_target, Py_target, l1, l2)

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
swingarmsensor = robot.getDevice('swingarm_pos_sensor')
swingarmsensor.enable(timestep)
wheelsensor= robot.getDevice('front_wheel_jounce_sensor')
wheelsensor.enable(timestep)
rear_wheel_sensor= robot.getDevice('rear_wheel_pos_sensor')
rear_wheel_sensor.enable(timestep)

####### INITIALIZE VALUES FOR STEERING CONTROL ########
T = 0

simtime = 0.0
yawRate = 0
oldYaw = 0

rollInt = 0
inteYawRate = 0
oldRoll = 0
R = 10
D = 20+1.45#+2.24+1.5
steerangle = 0
oldsteer = 0
steerRate = 0
goalroll=0
#FSM
Straight = True
Takingoff = False
Flying = False
Landing = False
theta_t = 0.7
theta_l = 0.349
# Kp =650
# Ki = 1300
Kd =.05#953#363# 650
Kp = 200#4765#3630#13000
Kp_speed= 5
Kd_speed= 1
ksum = 0.5
g = 9.81
m = 0
integral_error = 0
thetadot_des = 0
Torque_flying = 0
error = 0
previous_error = 0
swingarm_length = 0.5313
oldRoll,oldPitch,oldYaw = imu.getRollPitchYaw()
#flag to indicate first loop through the sim
firstLoop = True
#set the simulation forward speed and calculate rear wheel omega
Rrw = MC_params[10]
#driveOmega = driveVelocity/Rrw

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1 and simtime<=12:
    simtime+=timestep/1000.0
    if(firstLoop):
        oldRoll,oldPitch,oldYaw = imu.getRollPitchYaw()
        oldYaw = yawCorr.update(oldYaw)
        oldsteer = steersensor.getValue()
        firstLoop=False
    m = MC_params[4]+MC_params[7]+MC_params[9]+MC_params[11]+70
    # Read the sensors:
    #get current fwd speed
    U = gps.getSpeed()
    #get GPS reading
    xyz = gps.getValues()
    x = xyz[0]
    sensorValue = wheelsensor.getValue()
    #get IMU values and process to get yaw, roll rates
    #read IMU value
    rpy = imu.getRollPitchYaw()
    #gyros = gyro.getValues()
    gyros = gyro.getValues()
    #accels = accel.getValues()
    #ax = accels[0]
    yaw = rpy[2]
    yaw = yawCorr.update(yaw)
    yawRate = gyros[2]
    pitchRate = -gyros[1]
    pitch = -rpy[1]
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
    driveOmega = driveVelocity/(MC_params[10])
    #now set to that velocity (TODO make speed controller!)
    

    #update goal Y as appropriate:
    stepVal = 0
    if(simtime>=step_time):
        stepVal = step_mag

    #do control only if enough time has passed
    if 1:#((simtime-lastControlTime)>dTcontrol):
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
        #print('Steering Torque is: ')
        #print(eY)
            
    J = mff*(xff-a)**2+mrw*a**2+mfw*(b-a)**2+Jyyf+Jyyr
    angle = swingarmsensor.getValue()
    suspension_com=(angle) * swingarm_length
    ################### IMPLEMENT THE FSM ##########
    #BLOCK 1
    x = xyz[0]
    S=rear_wheel_sensor.getValue()*Rrw
    tf = D/(U*cos(theta_t)) #flying time 
    thetadot_des = -(theta_t+theta_l)/tf
    theta_des = -theta_l
    error_speed = Udes-U
    derivative_error_speed = (error_speed-previous_error)/(timestep/1000.0)
    #thetadot_des = 0
    #BLOCK 2
    T1 = Straight and (x<33.6)
    T2 = Straight and (x>=33.6)
    T3 = Takingoff and (x<34.9)
    T4  = Takingoff and (x>=34.9)
    T5 = Flying and (x<34.9+D)
    T6 = Flying and (x>=34.9+D)
    T7 = Landing and True 
    #BLOCK 3
    Straight = T1 
    Takingoff = T3 or T2
    Flying = T5 or T4
    Landing = T6 or T7 
   
    
    #BLOCK 4
    if (Straight): 
        Torque_speed = Kp_speed*error_speed#+Kd_speed* derivative_error_speed
        clampVal  = 300
        Torqueclamped = clamp(Torque_speed,-clampVal,clampVal)
        motor.setTorque(Torqueclamped)
        knee_motor.setPosition(theta1-theta1_rest)
        hip_motor.setPosition(theta2-theta2_rest)
        #print("setting vel directly")
        #motor.setVelocity(driveOmega)
    elif(Takingoff):
        T = 0
        thetaddot = ((thetadot_des)*U)/b
        ap = a/cos(theta_t)-h/sin(theta_t)
        J = m*(a**2+h**2)
        Torque_takingoff = ((J*thetaddot-m*g*ap)*Rrw)/h
        Torqueclamped = clamp(Torque_takingoff,-650,650)
        motor.setTorque(Torqueclamped)
        knee_motor.setPosition(theta1-theta1_rest)
        hip_motor.setPosition(theta2-theta2_rest)
    elif(Flying): 
        #Kp control
        error = theta_des - pitch
        derivative_error = 0- pitchRate
        derivative_error = (error-previous_error)/(timestep/1000.0)
        Torque_flying = ksum*(Kp * error + Kd * derivative_error)
        knee_motor.setPosition(theta1-theta1_rest)
        hip_motor.setPosition(theta2-theta2_rest)
        #KI contorl
        #error =thetadot_des-pitchRate # error for theta_dot in rad/s
        #integral_error = integral_error + error * (timestep/1000.0)
        #Torque_flying = ksum*(Kp * error + Ki * integral_error)
        Torqueclamped = clamp(Torque_flying,-650,650)
        motor.setTorque(Torqueclamped)
    elif(Landing): 
        motor.setVelocity(driveOmega)
        knee_motor.setPosition(theta1-theta1_rest)
        hip_motor.setPosition(theta2-theta2_rest)
    # print("speed = "+str(U)+", Tq: "+str(T)+", prev error: "+str(prev_y[0])+", yaw error: "+str(yawError)+", yaw: "+str(yaw)+", goalYaw: "+str(goalYaw) )
    #print("U: "+str(U)+", roll: "+str(roll)+", steer: "+str(steerangle)+", T="+str(T))
    # print("Torque: "+str(T))
    steer.setControlPID(0.0001,0,0)
    steer.setPosition(float('inf'))
    # WEBOTS is in ISO (z up) but Papa is in SAE (z down) so need to flip dir.
    steer.setTorque(T)
    lastControlTime = simtime
    error_speed = previous_error
    #print(U,thetadot_des,Torqueclamped,error,pitch,Torque_flying,Straight,Takingoff,Flying,Landing)
    if(recordData and simtime<=12):
        f.write(str(simtime)+","+str(pitchRate)+","+str(pitch)+","+str(U)+","+str(Torqueclamped)+","+str(suspension_com)+"\r\n")
f.close()
#makePlot()