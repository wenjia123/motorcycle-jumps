from controller import Supervisor

TIME_STEP = 32

robot = Supervisor()  # create Supervisor instance

# [CODE PLACEHOLDER 1]
mc_node = robot.getFromDef('PTW')


i = 0
simtime=0

#while robot.step(TIME_STEP) != -1:
while simtime<=12:
    simtime+=TIME_STEP/1000.0
    #if(i <5):
    mc_node.setVelocity([4.35,0,0,0,0,0])
    #i += 1
