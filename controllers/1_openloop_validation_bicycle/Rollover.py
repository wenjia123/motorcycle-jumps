import numpy as np
from matplotlib.pyplot import *

class Rollover:
    def __init__(self,ang = 0):
        self.angold = ang
        self.ang_corr = ang
        self.flips = 0
    def update(self,ang):
        if((ang-self.angold)<-3):
            self.flips+=1
        elif((ang-self.angold)>3):
            self.flips-=1

        self.ang_corr = ang+self.flips*2*np.pi
        self.angold = ang
        return self.ang_corr



#demo of class below. Only runs if you run file as a script
if __name__=='__main__':
    #create a scenario where a point moves in a circle
    r = 1
    #create a true vector of angles that goes up to 2*pi, then back down
    theta_true = np.hstack((np.arange(0,4*np.pi,.1),np.arange(4*np.pi,-4*np.pi,-.1)))

    #create X and Y vectors
    X = r*np.cos(theta_true)
    Y = r*np.sin(theta_true)

    #create an instance of the class
    rollCorrect = Rollover(0)

    #vector to hold raw and corrected detected angles
    theta_detect_raw = np.zeros(np.size(theta_true))
    theta_detect_corr = np.zeros(np.size(theta_true))

    #loop through and detect an angle from X and Y, and use class to track
    for k in range(0,len(theta_true)):
        x = X[k]
        y = Y[k]
        #use x,y to detect an angle
        theta_detect_raw[k] = np.arctan2(y,x)
        #use the class to correct it
        theta_detect_corr[k] = rollCorrect.update(theta_detect_raw[k])

    #now plot the results
    xvec = range(0,len(theta_true))
    figure()
    plot(xvec,theta_true,'ks',xvec,theta_detect_raw,'r',xvec,theta_detect_corr,'c')
    xlabel('index')
    ylabel('angle (rad)')
    legend(['true','detected','rollover corrected'])
    show()
