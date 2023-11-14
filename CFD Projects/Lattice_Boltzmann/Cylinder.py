#!/usr/bin/python3

import numpy as np
from numpy import save
from numpy import savetxt
import matplotlib.pyplot as plt
from matplotlib import cm
from scipy import signal
from time import process_time # includes function: time.time()

# general flow constants
lx     = 400;                    # number of cells in x-direction
ly     = 100;                    # number of cells in y-direction

obst_x = lx/5+1;                 # position of the cylinder
#print(obst_x)
obst_y = ly/2+3;                 # (exact y-symmetry is avoided)
obst_r = ly/10+1;                # radius of the cylinder
D = 2*obst_r;
pos_x = obst_x + (10*D);
pos_y = obst_y 
print(pos_x)
uMax   = 0.1;
                    # maximum velocity of Poiseuille inflow / Ma=uMax/sqrt(3)=0.0577
Re     = 50;                    # Reynolds number
nu     = uMax * 2.*obst_r / Re;  # kinematic viscosity
omega  = 1.0 / (3*nu+1./2.);     # relaxation parameter

maxT   = 30000;                  # total number of iterations
tPlot  = 1000;                   # cycles


#----------------------------------------------------


# D2Q9 LATTICE CONSTANTS 
t  = np.array([4/9, 1/9,1/9,1/9,1/9, 1/36,1/36,1/36,1/36]) 
c = np.array([[0, 1, 0, -1,  0, 1, -1, -1,  1],
              [0, 0, 1,  0, -1, 1,  1, -1, -1]]).T

opp = np.array([ 0, 3, 4, 1, 2, 7, 8,  5,  6])


# defining the boundaries
obst = np.fromfunction(lambda x,y: (x-obst_x)**2+(y-obst_y)**2 < obst_r**2, (lx,ly)) # location of the cylinder

obst[:, 0] = 1 # Bottom
obst[:,-1] = 1 # Top
bbRegion = np.where(obst)

# equilibrium distribution function
def equilibrium(rho,u):              
    cu   = 3.0 * np.dot(c,u.transpose(1,0,2))
    usqr = 3./2.*(u[0]**2+u[1]**2)
    feq = np.zeros((9,lx,ly))
    for i in range(9): feq[i,:,:] = rho*t[i]*(1.+cu[i]+0.5*cu[i]**2-usqr)
    return feq

# initial condition: Poiseuille profile at equilibrium
u = np.fromfunction(lambda d,x,y: (1-d)*4*uMax /((ly-2)*(ly-2)) *\
                    ((y-0.5)*(ly-2)-(y-0.5)*(y-0.5)),(2,lx,ly))
rho = 1.0
fEq = equilibrium(rho,u)
fIn = fEq.copy()

veloPointProbe = []

# timeloop
for timeStep in range(maxT):
    t1_start = process_time()
    #macroscopic variables
    rho = np.sum(fIn, axis=0)
    u = np.dot(c.transpose(), fIn.transpose((1,0,2)))/rho
    veloPointProbe.append(u[1, int(pos_x),int(pos_y)] )
    #veloPointProbe = u[1, int(pos_x):,int(pos_y)] 
    # inlet: Poiseuille profile
    u[:,0,1:-1] = np.fromfunction(lambda d,y: (1-d)*4*uMax /((ly-2)*(ly-2)) *\
                        ((y-0.5)*(ly-2)-(y-0.5)*(y-0.5)) ,(2,ly-2))
        
    rho[0,1:-1] = 1/(1-u[0,0,1:-1]) * (np.sum(fIn[[0,2,4],0,1:-1], axis=0 ) + 2 * np.sum(fIn[[3,6,7],0,1:-1], axis=0 ))
    
    # outlet: constant pressure 
    rho[-1,1:-1] = 1.0
    
    u[0,-1,1:-1] = -1 + 1/rho[-1,1:-1] * (np.sum(fIn[[0,2,4],-1,1:-1], axis=0 ) + 2 * np.sum(fIn[[1,5,8],-1,1:-1], axis=0 ))
    u[1,-1,1:-1] = 0.0

    # microscopic boundary conditions Zou/He inlet
    fIn[1,0,1:-1] = fIn[3,0,1:-1] + 2/3*rho[0,1:-1]*u[0,0,1:-1]
    
    fIn[5,0,1:-1] = fIn[7,0,1:-1] + 1/2*(fIn[4,0,1:-1]-fIn[2,0,1:-1])\
                                  + 1/2*rho[0,1:-1]*u[1,0,1:-1]\
                                  + 1/6*rho[0,1:-1]*u[0,0,1:-1]

    fIn[8,0,1:-1] = fIn[6,0,1:-1] + 1/2*(fIn[2,0,1:-1]-fIn[4,0,1:-1])\
                                  - 1/2*rho[0,1:-1]*u[1,0,1:-1]\
                                  + 1/6*rho[0,1:-1]*u[0,0,1:-1]
                                  
    # microscopic boundary conditions Zou/He outlet
    fIn[3,-1,1:-1] = fIn[1,-1,1:-1] - 2/3*rho[-1,1:-1]*u[0,-1,1:-1]
    
    fIn[7,-1,1:-1] = fIn[5,-1,1:-1] + 1/2*(fIn[2,-1,1:-1]-fIn[4,-1,1:-1])\
                                    - 1/2*rho[-1,1:-1]*u[1,-1,1:-1]\
                                    - 1/6*rho[-1,1:-1]*u[0,-1,1:-1]
    
    fIn[6,-1,1:-1] = fIn[8,-1,1:-1] + 1/2*(fIn[4,-1,1:-1]-fIn[2,-1,1:-1])\
                                    + 1/2*rho[-1,1:-1]*u[1,-1,1:-1]\
                                    - 1/6*rho[-1,1:-1]*u[0,-1,1:-1]    
    
    # collision step
    fEq = equilibrium(rho,u)
    fOut = fIn - omega * (fIn - fEq)
    
    # microscopic boundary condition (bounce back)
    for i in range(9): 
        fOut[i][bbRegion] = fIn[opp[i]][bbRegion]
        
    # streaming step
    for i in range(9):
        fIn[i,:,:] = np.roll(np.roll(fOut[i,:,:],c[i,0],axis=0),c[i,1],axis=1)
    
    #print(veloPointProbe)
    t2_stop = process_time()
    T = t2_stop - t1_start              #Simulation Time
    #print("Process Time: ", t2_stop-t1_start)   
    
    # visualisation of the velocity field
    if (timeStep%tPlot ==0):
        u[0][bbRegion] = np.NaN
        plt.clf(); plt.imshow(np.sqrt(u[0]**2+u[1]**2).transpose()/uMax,
                              cmap=cm.jet, origin='lower', vmin=0, vmax =1.5)
        plt.savefig("veloMag2D_Cyl_"+str(int(timeStep/tPlot))+".png")     
	#plt.show()
    


savetxt('data2.csv', veloPointProbe, delimiter=',')


# fs = 1

# (f, S)= signal.welch(initial, fs, nperseg=1024)
# plt.semilogy(f, S)
# plt.xlabel('frequency [Hz]')
# plt.ylabel('PSD [V**2/Hz]')
# plt.show()

# max_S = max(S)
# max_fs = f[S.argmax()]
# St = (max_fs*D)/uMax
