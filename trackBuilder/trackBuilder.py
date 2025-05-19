from numpy import *
from matplotlib.pyplot import *


class Track:

    def __init__(self,lengths,angles,dS=.1):
        self.dS = dS
        self.Scoarse = array([0])
        self.Kcoarse = array([0])
        self.buildTrack(lengths,angles)

    def fillTrack(self):
        nu = array([0])
        X = array([0])
        Y = array([0])
        #build a high resolution map
        Sq = arange(0,max(self.Scoarse),self.dS);
        Kq = interp(Sq,self.Scoarse,self.Kcoarse);


        for k in range(1,len(Sq)):
           nu = append(nu,nu[k-1]+Kq[k-1]*(Sq[k]-Sq[k-1]))
           X = append(X,X[k-1] + (Sq[k]-Sq[k-1])*cos(nu[k-1]))
           Y = append(Y,Y[k-1] + (Sq[k]-Sq[k-1])*sin(nu[k-1]))


        self.heading = nu
        self.X = X
        self.Y = Y
        self.S = Sq
        self.K = Kq


    def addStraight(self,lengthStraight):
        #for purposes of interpolation, need to make sure S changes a tiny bit.
        fudge = 0.001;
        #append S with last S plus "a little bit"
        self.Scoarse = append(self.Scoarse,self.Scoarse[-1]+fudge)
        #append K with the curvature for a straight segment (should be 0)
        self.Kcoarse = append(self.Kcoarse,0)
        #append S with the length of the straight + last S.
        self.Scoarse = append(self.Scoarse,self.Scoarse[-1]+lengthStraight)
        #straight segment should mean that second S point should also have this curvature
        self.Kcoarse = append(self.Kcoarse,0)


    def addTurn(self,RTurn,thetaTurn):
        fudge = 0.001
        Kturn = 1.0/RTurn
        dTheta = self.dS/RTurn
        STurn = abs(thetaTurn*RTurn)
        #for purposes of interpolation, need to make sure S changes a tiny bit.
        self.Scoarse = append(self.Scoarse,self.Scoarse[-1]+fudge)
        #append S coarse vector with the length of the arc of this turn
        self.Scoarse = append(self.Scoarse,self.Scoarse[-1]+STurn);

        if thetaTurn <=0:
            self.Kcoarse = append(self.Kcoarse,-Kturn)
            self.Kcoarse = append(self.Kcoarse,-Kturn)
        else:
            self.Kcoarse = append(self.Kcoarse,Kturn)
            self.Kcoarse = append(self.Kcoarse,Kturn)

    def StraightOrTurn(self,Length,angle):
        if angle != 0:
            #this means that the length should be interpreted as a radius!
            self.addTurn(RTurn=Length,thetaTurn=angle);
        else:
            #this means that the length should be a straightaway
            self.addStraight(lengthStraight=Length)

    def buildTrack(self,lengths,angles):
        for i in range(0,len(angles)):
            self.StraightOrTurn(lengths[i],angles[i])
        self.fillTrack()


if __name__=='__main__':
    #build a track that is a simple oval with two 180 degree turns. Each will have a radius of 25 meters, and the straights will be 200m long.
    lengths = [200,25,200,25]
    angles = [0,pi,0,pi]

    track = Track(lengths,angles)

    figure()
    plot(track.X,track.Y,'k')
    xlabel('X (m)')
    ylabel('Y (m)')
    axis('equal')

    show()
