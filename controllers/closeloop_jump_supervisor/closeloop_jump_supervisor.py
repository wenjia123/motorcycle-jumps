from controller import Supervisor

TIME_STEP = 1

robot = Supervisor()  # create Sfile:///Users/wenjia/Library/CloudStorage/GoogleDrive-liwe@lafayette.edu/.shortcut-targets-by-id/1N9xpN0QFZIMU-kw6Avf2NYt0l5AkwvQu/ES302_Li/final_poroject/RSS2024/controllers/upervisor instance

# [CODE PLACEHOLDER 1]
mc_node = robot.getFromDef('PTW')


i = 0
while robot.step(TIME_STEP) != -1:
  if (i <1000):
    #print(mc_node)
    mc_node.setVelocity([0,0,0,0,0,0])
    #17
  #if i % 500 == 0:
       #robot.exportImage('../scripts/Images/Image_' + str('PI')+str(i) + '.jpg',90)
    
  i += 1
  