from trackBuilder import Track
from numpy import *
from matplotlib.pyplot import *

#define our track based on a list of length values and a list of angle values.
#for a straight, the entry in lengths means how long the straight is, and
#the corresponding angles entry should be 0. For a turn, the length value
#is the turn radius, and the angle value is the subtended angle of the turn.

lengths = [25,10,25,10,12.5,10,33,10,19.75];
angles = [0,pi,0,-pi/2+.02,0,pi,0,pi/2,0];

#now use the trackbuilder's Track class to define the track object.
track = Track(lengths, angles, dS = 0.8)

#now, format the track object for inclusion in the Webots world file.
#should be a single string of "[x1 y1 z1, x2 y2 z2, ... ,xn yn zn]"
webotsString = "wayPoints [" #this starts the line off. Now use a for-loop:
maptoolsString = ""
for k in range(0,len(track.S)-1):
    #append with this point
    webotsString+= str(track.X[k])+" "+str(track.Y[k])+" "+str(0)+","
    maptoolsString+= str(track.X[k])+","+str(track.Y[k])+","+str(0)+"\r\n"

#now close it out.
webotsString+=str(track.X[-1])+" "+str(track.Y[-1])+" "+str(0)
webotsString+="]\r\n"

#now write this track to the webots world file "track.wbt"
worldFileName = "../worlds/track.wbt"
maptoolsFileName = "../map/track.csv"
#read file into memory:
with open(worldFileName,'r',encoding="utf-8") as file:
    data = file.readlines()
#find the line that defines the waypoints and replace it with our new ones:
print(data)
for k in range(0,len(data)):
    if "wayPoints ["  in data[k]:
        kPoints = k
print(data[kPoints])
data[kPoints] = webotsString
print(data[kPoints])

#overwrite webots world file with new track data.
with open(worldFileName,'w',encoding='utf-8') as file:
    file.writelines(data)
file.close()

#overwrite maptools file with track data.
with open(maptoolsFileName,'w',encoding='utf-8') as file:
    file.write(maptoolsString)
file.close()


#now, plot the track to make sure it looks right.
figure()
plot(track.X,track.Y,'k')
xlabel('X (m)')
ylabel('Y (m)')
axis('equal')
show()
