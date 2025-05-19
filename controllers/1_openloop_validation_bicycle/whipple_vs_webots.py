from numpy import *
from matplotlib.pyplot import *
#rcParams['text.usetex'] = True
from scipy import signal
import control
import control.matlab as cnt
sys.path.insert(0, '../Models')
from whipple_model import *
import matplotlib.pyplot as plt
#construct Whipple model of the bike.
param_names = ['a ','b ','c','hrf','mrf','xff','zff','mff','Rfw','mfw','Rrw','mrw','Jyyf','Jyyr','lam']
# params = array([.3,1.02,.08,.9,85,.9,.7,4,.35,3,.3,3,.28*.65,.12*.65,1.25])
params = array([.3,1.02,.08,.9,85,.9,.7,4,.35,3,.3,3,3*.35**2,3*.3**2,1.25])

def getEigsVecs(params):
    #create a vector of velocities to investigate
    vvec = arange(.01,10,.01)
    #create a second copy of this vector that is four rows. This will make plotting easier
    vvec2 = zeros((4,len(vvec)))

    #our model is fourth order, so will have 4 eigenvalues. each can have a real and/or imaginary part.
    eigs_re = zeros((4,len(vvec)))
    eigs_im = zeros((4,len(vvec)))

    for k in range(0,len(vvec)):
        #get current velocity
        v = vvec[k]
        #get state space model at this speed
        sys= getModelSS(v,params)
        #get eigenvalues at this speed
        eigs,vecs = linalg.eig(sys.A)
        #get real parts and place in proper matrix for storage
        eigs_re[:,k] = real(eigs)
        #get imaginary parts and place in proper matrix for storage
        eigs_im[:,k] = imag(eigs)
        #fill up velocity vector corresponding with each eigenvalue
        vvec2[:,k] = [v,v,v,v]
    return vvec,eigs_re,eigs_im
######################## EIGS VS SPEED ##########################


def makePlot():
    vvec, eigs_re, eigs_im = getEigsVecs(params)
    figure()
    plot(vvec,eigs_re[0,:],'k.',vvec,eigs_im[0,:],'k')
    xlabel('Speed (m/s)')
    ylabel('Eigenvalue (1/s)')
    legend(['real','imaginary'])
    plot(vvec,eigs_re[1,:],'k.',vvec,abs(eigs_im[1,:]),'k')
    plot(vvec,eigs_re[2,:],'k.',vvec,abs(eigs_im[2,:]),'k')
    plot(vvec,eigs_re[3,:],'k.',vvec,abs(eigs_im[3,:]),'k')
    ylim([-10,10])
    ######################## STEP RESPONSE ##########################

    #load data file from webots:
    t,spd,tq,roll,rollrate,steer,steerrate = loadtxt("step_data.txt",delimiter=",",unpack=True)
    U = mean(spd)#what was the speed of the test
    T = mean(tq)#what was the step magnitude?
    X0 = array([roll[0],steer[0],rollrate[0],steerrate[0]])
    print("Testing at velocity "+str(U)+" and step torque "+str(T))




    #get state space model of bike based on Whipple
    sys = getModelSS(U,params)
    #perform an lsim using measured torque and initial condition values
    yout,tout,xout = cnt.lsim(sys,tq,t,X0)


    #now plot data vs. whipple
    figure()
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
    subplot(2,1,1)
    ax1.plot(tout,yout[:,0],'k',t,roll,'r')
    legend(['linear model','Webots'],fontsize="13")
    #title('$U=$ '+str(round(U,2))+"m/s; $T_\delta=$ "+str(round(T,2))+"Nm; $\phi_0=$"+str(round(roll[0],2))+" rasd")
    title('A',fontsize = 15)

    ylabel('Roll (rad)',fontsize=14)
    plt.yticks(fontsize = 15)
    #plt.xticks([])
    subplot(2,1,2)
    ax2.plot(tout,yout[:,1],'k',t,steer,'r')
    ylabel('Steer (rad)',fontsize=14)
    xlabel('Time (s)',fontsize=14)
    plt.yticks(fontsize = 15)
    plt.xticks(fontsize = 15)
    fig.align_ylabels()
    plt.tight_layout()
    plt.savefig("../../scripts/Figures/1_whipple_vs_Webots_phi0_"+str(round(roll[0],2))+" rad.png",dpi=1000)
    show()
