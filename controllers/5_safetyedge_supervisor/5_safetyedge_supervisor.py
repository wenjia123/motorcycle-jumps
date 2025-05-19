from controller import Supervisor
from controller import Node
from numpy import arange,savetxt
# 0, 1, 2, 3, 4, check where code is stopping and not runnig
# get access to nodes through Supervisor

TIME_STEP = 1
robot = Supervisor()

# mc_node = robot.getFromDef('PTW')
recordData = True
# get our safety edge node
safedge_node = robot.getFromDef('SlantRoad')
road_pos_field = safedge_node.getField('roadPosition')
slant_field = safedge_node.getField('angle')
print("road position field is: " +str(road_pos_field))

# get our safety edge robot supervisor node
safe_edge_rob = robot.getFromDef('safedge_rob')
print("Safety edge supervisor robot is: "+str(safe_edge_rob))

# get out webikes proto node
mc_node = robot.getFromDef('PTW')
# print("motorcycle robot is: "+str(dir(mc_node)))


print('Road Position Initially is:') # this is being printed
road_pos = road_pos_field.getSFVec3f() # creates a road pos vector of length 3
print(road_pos) # this is being printed

print('Slant angle initially is:')
slant_angle = slant_field.getSFFloat()
print(slant_angle)

# create and set all variables needed for simulation
sim_counter = 0 # counts number of sims occured
sim_time = 0 # updates to match time step
sim_time_max = 10 # max time we want simulation to update
offset_values = arange(0,4,.25)#[0.50, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0]
slant_values = [30,90]


i = 0 # index for offset_values array
rp_x = 0 # x pos of road vec
rp_y = 0 # y pos of road vec
rp_z = 0 # z pos of road vec

# sets each index of vector equal to those
road_pos[0] = rp_x
road_pos[1] = rp_y
road_pos[2] = rp_z


print('vars') # this is being printed
print(road_pos) # this is being printed

sim_counter = 0
slant_counter = 0
recordVideos = True
#open a file that will record the simulation index.

#this file gets read by the PTW controller to organize how it saves data.
if(recordData):
    y = open('../../scripts/casestudy_data/road_position_index.txt', 'w') # opens txt for file
    y.write(str(sim_counter)) # writes the initial y position to file
    y.close() # closes file...
    print('initial printed to text file')

    #now write a file that records the offset values
    savetxt('../../scripts/casestudy_data/road_positions.txt',offset_values)
    #now write the current slant angle to a file.
    slant_file = open('../../scripts/casestudy_data/edge_slant.txt','w')
    slant_file.write(str(int(slant_values[slant_counter])))
    slant_file.close()

for slant_counter in range(0,len(slant_values)):
    for sim_counter in range(0,len(offset_values)): # replaced while with for loop
            
            print("Offset Counter is: "+str(sim_counter))
            print("Slant Counter is: "+str(slant_counter))
            rp_y = 0 # resets y for each sim so offsets incs correctly
            offset = offset_values[sim_counter]
            new_rp_y = rp_y + offset # made new var to avoid pot issues
            new_road_pos = [0, new_rp_y, 0] # we need to put new y values back into vec

            road_pos_field.setSFVec3f(new_road_pos) # sets original road_pos to new one
            road_pos = new_road_pos # testing if assignin vecs like this solves issue

            slant_field.setSFFloat(slant_values[slant_counter])
            print(new_road_pos) # this is being printed
            print(road_pos) # this is being printed! theybboth match!
            #set road position
            if(recordData):
                y = open('../../scripts/casestudy_data/road_position_index.txt', 'w') # opens txt for file
                y.write(str(sim_counter)) # writes new val to txt file, # str(val)
                y.close()
                slant_file = open('../../scripts/casestudy_data/edge_slant.txt','w')
                slant_file.write(str(slant_values[slant_counter]))
                slant_file.close()
                print('printed to the txt files') # keeps rewriting over previous values, this is a later fix
            if(recordVideos):
                vidname = '../../videos/slant_'+str(slant_values[slant_counter])+'_offset_'+str(offset*100)+'.mp4'
                cap = 'speed: 10 m/s, edge angle: '+str(slant_values[slant_counter])+', offset: '+str(offset)+'m'
                robot.movieStartRecording(vidname,1280,720,0,100,1,False)
            simsteps = 0
            #create a 'local' sim time variable that keeps track of THIS simulation's time only.
            current_simtime = 0
            # need to make sure robot time is reset
            while (robot.step(TIME_STEP)!=-1) and (current_simtime<sim_time_max): # implements correctly
                #increment our 'local' simulation time by one timestep converted to seconds from ms
                current_simtime+=TIME_STEP/1000.0
                #print(current_simtime)
                #at beginning of simulation, give bike proper initial x velocity
                if(simsteps<20):
                     mc_node.setVelocity([15.6,0,0,0,0,0])
                     road_pos_field.setSFVec3f(new_road_pos) # sets original road_pos to new one

                #increment how many simulation steps we've done.
                simsteps+=1

                # robot.step(TIME_STEP) # increments time appropriately
                #print('time check while loop in progress')
                #print(mc_node.getTime())
                #print(robot.getTime())

            print('Exit while loop')

            # initial 000 vector to set road_pos back to
            # init_road_pos = [0, 0, 0]
            # road_pos_field.setSFVec3f(init_road_pos) # ressets road position vec (might not need)
            # print(road_pos_field.setSFVec3f(init_road_pos))

            # NTP: needs to reset before reloading
            robot.simulationReset() # ONLY resets siulation, not robot simulation time
            if(recordVideos):
                robot.movieStopRecording()
                while not robot.movieIsReady():
                    pass
             # restart the safety edge node controller
            # safe_edge_rob.restartController() # resets robot controller

            # restart bike controller as well (might be impacting sim)
            mc_node.restartController()

            # CODE BLOCK E
            #robot.worldReload()
            #print('Reset Sim')

             # reset world,simulation, Supervisor controller, reset vector
            # robot.worldSave() # saves before annoying pop up

             # CODE BLOCK S <-- needs to occur before while loop!!!
            i+=1 # updates index of offset values array
            #do not increment sim counter. for-loops do this automatically
            # sim_counter+=1 # incremts sim counter AFTER exiting loop
            # print('s ends with a current value of '+ str(i))
            # print('i ends with a current value of ' +str(i))
            # print(sim_counter)
            # print('e')

                     #if (i <20 ):
                    #mc_node.setVelocity([10,0,0,0,0,0])
                    #i += 1

                    # code moves t
