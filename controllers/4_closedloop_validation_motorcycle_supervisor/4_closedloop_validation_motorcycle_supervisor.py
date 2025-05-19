from controller import Supervisor

TIME_STEP = 32

robot = Supervisor()  # create Supervisor instance

# [CODE PLACEHOLDER 1]
mc_node = robot.getFromDef('PTW')


i = 0
while robot.step(TIME_STEP) != -1:
  if (i <20 ):
    mc_node.setVelocity([15.6,0,0,0,0,0])
  i += 1
