import environments
import solvers
import airfoils
import aircraft
import numpy as np
import math

import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

# define the simulation environment
dt, T = 0.01, 10
rho = 1.225
G = [0,0,9.81]
env = environments.Environment(dt,T,rho,G)

# define the aircraft state, mass, cg, moment of inertia
inertialPos = [0,0,0]
inertialAtt = [0,0,0]
bodyLinVel = [15, 0.5, 1]
bodyLinAcc = [0, 0, 0]
bodyAngVel = [0, 0, 0]
bodyAngAcc = [0, 0, 0]
cg = [0,0,0]
I = [[1,0,1],[0,1,0],[1,0,1]]
vehicle = aircraft.Vehicle(env,inertialPos,inertialAtt,bodyLinVel,bodyLinAcc,bodyAngVel,bodyAngAcc,cg,I)

# define the wings
L1 = [0, 0, 0]
L2 = [-0.9*math.atan(15*np.pi/180), -0.9, 0.]
L3 = [-0.9*math.atan(15*np.pi/180)-0.11, -0.9 ,0.]
L4 = [-0.25, 0, 0]
COORDSLW = [L1, L2, L3, L4]
R1 = [0, 0, 0]
R2 = [-0.9*math.atan(15*np.pi/180), 0.9, 0.]
R3 = [-0.9*math.atan(15*np.pi/180)-0.11,0.9,0.]
R4 = [-0.25, 0, 0]
COORDSRW = [R1, R2, R3, R4]
leftwing = aircraft.Wing(vehicle, COORDSLW, 0.9)
rightwing = aircraft.Wing(vehicle, COORDSRW, 0.9)
vehicle.addWing(leftwing)
vehicle.addWing(rightwing)

# define the solvers
solverlw = solvers.StripTheorySolver(leftwing,airfoils.IdealAirfoil(), 5)
solverrw = solvers.StripTheorySolver(rightwing, airfoils.IdealAirfoil(), 5)

# add solvers to wings
leftwing.addSolver(solverlw)
rightwing.addSolver(solverrw)

# solve for forces
vehicle.forces()
vehicle.coefficients()

# plot the vehicle
fig = plt.figure()
ax = fig.gca(projection='3d')
vehicle.plotComponents(ax)
ax.set_xlim3d(-1,1)
ax.set_ylim3d(-1,1)
ax.set_zlim3d(-1,1)

# set axis labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Turn off tick labels
ax.set_xticklabels([])
ax.set_yticklabels([])
ax.set_zticklabels([])

plt.show()