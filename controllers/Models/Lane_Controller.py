from numpy import *
from matplotlib.pyplot import *
from scipy import signal
import control
from whipple_model import *

def getLQR(v,Q,R,params):
    sys = getModelSS(v,params)
    K,S,E = control.lqr(sys,Q,R)
    syscl = control.StateSpace(sys.A-dot(sys.B,K),vstack([1,0,0,0]),eye(4),0)
    return K,syscl

###### LQR and state space system with yaw angle as an (augmented) state. ######
def getModelSSYaw(v,params):
    #first get the model in MDK form
    M,D,K,F = getModelMDK(v,params)
    A = hstack((zeros((2,2)),eye(2,2)))#first create the 'top half' of A matrix
    A = vstack((A,hstack((dot(-linalg.inv(M),K),dot(-linalg.inv(M),D)))))# now fill in bottom half
    #augment A to add yaw as a state
    A = hstack((A,zeros((4,1))))
    A = vstack((A,zeros((1,5))))
    A[4,1] = v*sin(params[14])/params[1]
    # A[4,3] = params[2]*sin(params[14])/params[1]

    B = vstack((zeros((2,2)),dot(linalg.inv(M),F)))#now stack zeros on top of our M^-1F term
    #trim B so it only takes steer.
    B = vstack(B[:,1])
    #stack B with a row of zero for augmented (yaw) state
    B = vstack((B,0))

    C = array([[1,0,0,0,0],[0,1,0,0,0]]) #choose the 'outputs' as just roll and steer
    D = 0 #with two possible inputs, two possible outputs, there are four terms in the D matrix when output eqn is y = Cx+Du
    sys = control.StateSpace(A,B,C,D)
    return sys

def getLQRYaw(v,Q,R,params):
    sys = getModelSSYaw(v,params)
    K,S,E = control.lqr(sys,Q,R)
    syscl = control.StateSpace(sys.A-dot(sys.B,K),vstack([0,0,0,0,1]),eye(5),0)
    return K,syscl



########## State space and LQR for system augmented with roll error integral
def getModelSSI(v,params):
    #first get the model in MDK form
    M,D,K,F = getModelMDK(v,params)
    A = hstack((zeros((2,2)),eye(2,2)))#first create the 'top half' of A matrix
    A = vstack((A,hstack((dot(-linalg.inv(M),K),dot(-linalg.inv(M),D)))))# now fill in bottom half
    #augment A to add yaw as a state
    A = hstack((A,zeros((4,1))))
    A = vstack((A,zeros((1,5))))
    A[4,0] =-1
    # A[4,3] = params[2]*sin(params[14])/params[1]

    B = vstack((zeros((2,2)),dot(linalg.inv(M),F)))#now stack zeros on top of our M^-1F term
    #trim B so it only takes steer.
    B = vstack(B[:,1])
    #stack B with a row of zero for augmented (yaw) state
    B = vstack((B,0))

    C = array([[1,0,0,0,0],[0,1,0,0,0]]) #choose the 'outputs' as just roll and steer
    D = 0 #with two possible inputs, two possible outputs, there are four terms in the D matrix when output eqn is y = Cx+Du
    sys = control.StateSpace(A,B,C,D)
    return sys

def getLQRI(v,Q,R,params):
    sys = getModelSSI(v,params)
    K,S,E = control.lqr(sys,Q,R)
    syscl = control.StateSpace(sys.A-dot(sys.B,K),vstack([0,0,0,0,1]),eye(5),0)
    return K,syscl


########## State space and LQR for system augmented with roll error integral
def getModelSSy(v,params):
    # first get the model in MDK form
    M,D,K,F = getModelMDK(v,params)
    A = hstack((zeros((2,2)),eye(2,2)))#first create the 'top half' of A matrix
    A = vstack((A,hstack((dot(-linalg.inv(M),K),dot(-linalg.inv(M),D)))))# now fill in bottom half
    #augment A to add yaw as a state
    A = hstack((A,zeros((4,2))))
    A = vstack((A,zeros((2,6))))
    # yaw rate = U*sin(lam)/b*delta + c*sin(lam)/b*deltadot
    A[4,1] = v*sin(params[14])/params[1]
    A[4,3] = params[2]*sin(params[14])/params[1]
    # d(lateral position)/dt = U*psi
    A[5,4] = v

    B = vstack((zeros((2,2)),dot(linalg.inv(M),F)))#now stack zeros on top of our M^-1F term
    #trim B so it only takes steer.
    B = vstack(B[:,1])
    #stack B with a row of zero for augmented (yaw) state and augmented y state
    B = vstack((B,0,0))

    C = array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,0,0,0,1]]) #choose the 'outputs' as just roll and steer and lat pos
    D = 0 #with two possible inputs, two possible outputs, there are four terms in the D matrix when output eqn is y = Cx+Du
    sys = control.StateSpace(A,B,C,D)
    return sys

def getLQRy(v,Q,R,params):
    sys = getModelSSy(v,params)
    K,S,E = control.lqr(sys,Q,R)
    syscl = control.StateSpace(sys.A-dot(sys.B,K),dot(sys.B,K),eye(6),0)
    return K,syscl

######### OPEN LOOP EIGENVALUE STUDY
def plotEigStudy(params,doPlot=False):

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

    if(doPlot):
        figure()
        plot(vvec,eigs_re[0,:],'k.',vvec,eigs_im[0,:],'k',)
        xlabel('Speed (m/s)')
        ylabel('Eigenvalue (1/s)')
        legend(['real part','imaginary part'])
        title('Open-Loop Eigenvalues vs. speed')
        plot(vvec,eigs_re[1,:],'k.',vvec,eigs_im[1,:],'k')
        plot(vvec,eigs_re[2,:],'k.',vvec,eigs_im[2,:],'k')
        plot(vvec,eigs_re[3,:],'k.',vvec,eigs_im[3,:],'k')
        ylim([-10,10])


    return vvec,eigs_re,abs(eigs_im)
