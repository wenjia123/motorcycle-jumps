from numpy import *
from matplotlib.pyplot import *
import os, glob, re #for handling files
import matplotlib.image as mpimg


#load the offsets
offsets = loadtxt('casestudy_data/road_positions.txt')
offsets+=0.25#adjust for initial offset in world file
print("offsets are:")
print(offsets)

maxRoll90 = zeros(len(offsets))
maxTq90 = zeros(len(offsets))
maxSteer90 = zeros(len(offsets))
maxRoll30 = zeros(len(offsets))
maxTq30 = zeros(len(offsets))
maxSteer30 = zeros(len(offsets))
success30 = zeros(len(offsets))
success90 = zeros(len(offsets))

# figure()
for filename in glob.glob('casestudy_data/safetyedge_data_offset_*.txt'):
   with open(os.path.join(os.getcwd(), filename), 'r') as f: # open in readonly mode
      print(filename)
      #split filename to determine what kind of test this was
      fname_split = filename[0:-4].split('_')
      #what offset is this
      offsetind = int(fname_split[4])
      #what slant is this
      slantnow = int(fname_split[-1])
      #give us the data
      data = loadtxt(filename,delimiter=",")
      # data file structure:
      #str(simtime)+","+str(T)+","+str(U)+","+str(roll)+","+str(steerangle)+","+str(rollRate)+","+str(steerRate)+","+str(rollInt)+","+str(stepVal)+","+str(yaw)+","+str(latPos)+"\r\n")
      print(slantnow,offsetind)
      maxtq = max(abs(data[:,1]))
      maxsteer = max(abs(data[:,4]))
      maxroll = max(abs(data[:,3]))
      success = maxroll<1.0 #did the vehicle fall over or not
      print(maxtq,maxsteer,maxroll)
      if(slantnow == 90):
          if(success):
              maxTq90[offsetind] = maxtq
              maxSteer90[offsetind] = maxsteer
              maxRoll90[offsetind] = maxroll
          else:
              maxTq90[offsetind] = NaN
              maxSteer90[offsetind] = NaN
              maxRoll90[offsetind] = NaN
          success90[offsetind] = success
          # if(maxroll<1):
          #     subplot(3,2,1)
          #     plot(data[:,0],data[:,1],'k')
          #     xlabel('time (s)')

      elif(slantnow==30):
          if(success):
              maxTq30[offsetind] = maxtq
              maxSteer30[offsetind] = maxsteer
              maxRoll30[offsetind] = maxroll
          else:
              maxTq30[offsetind] = NaN
              maxSteer30[offsetind] = NaN
              maxRoll30[offsetind] = NaN
          success30[offsetind] = success

fig1 = figure(figsize=(16, 4), dpi=100)

subplot(1,4,1)
### FOR SUCCESS MATRIX USE BELOW
# plot(offsets,success90,'k-o',offsets,success30,'r-x',markerfacecolor='none',mew=2,markersize=12)
# grid('on')
# xlabel('edge lateral offset (m)',fontsize=14)
# ylabel('Success (0=Failure)',fontsize=14)
# legend(['90 degree edge','30 degree edge'],loc='right')
# title('a',fontsize=14)
####FOR EXAMPLE IMAGE USE BELOW
img=mpimg.imread('Figures/5_safetyedge_world_traversing.png')
imshow(img)
axis('off')
title('a',fontsize=14)

subplot(1,4,2)
axis('on')
plot(offsets,maxTq90,'ko',offsets,maxTq30,'rx',markerfacecolor='none',mew=2,markersize=12)
grid('on')
legend(['90 degree edge','30 degree edge'])
xlabel('edge lateral offset (m)',fontsize=14)
ylabel('maximum steer torque (Nm)',fontsize=14)
title('b',fontsize=14)
# legend(['90 degree edge','30 degree edge'])

# figure()
subplot(1,4,3)
plot(offsets,maxSteer90*180/pi,'ko',offsets,maxSteer30*180/pi,'rx',markerfacecolor='none',mew=2,markersize=12)
xlabel('edge lateral offset (m)',fontsize=14)
ylabel('maximum steer angle (degrees)',fontsize=14)
grid('on')
title('c',fontsize=14)
# legend(['90 degree edge','30 degree edge'])

# figure()
subplot(1,4,4)
plot(offsets,maxRoll90*180/pi,'ko',offsets,maxRoll30*180/pi,'rx',markerfacecolor='none',mew=2,markersize=12)
xlabel('edge lateral offset (m)',fontsize=14)
ylabel('maximum Roll angle (degrees)',fontsize=14)
grid('on')
title('d',fontsize=14)

fig1.tight_layout()
savefig("Figures/5_casestudy_panel1.png",dpi=1000)

show()
