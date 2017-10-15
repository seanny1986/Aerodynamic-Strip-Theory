import numpy as np
import math

class Environment():
    def __init__(self, dt, T, rho, G):
        self.dt = dt
        self.T = T
        self.rho = rho
        self.G = np.asarray(G)
        self.vehicles = []
        self.vehiclePosition = []

    def addVehicle(self, vehicle):
        self.vehicles.append(vehicle)

    def update(self):
        for v in self.vehicles:
            v.forces()
            v.stateupdate()

    def runSimulation(self):
        t = 0
        while t < self.T:
            self.update
            t += self.dt
    
    def inertialToBodyDirectional( self, ATTITUDE, HEADING):
        phi = ATTITUDE[0]
        theta = ATTITUDE[1]
        psi = ATTITUDE[2]

        ROTATION = np.array([[math.cos(theta)*math.cos(psi), math.cos(theta)*math.sin(psi), -math.sin(theta)],
                        [math.sin(phi)*math.sin(theta)*math.cos(psi)-math.cos(phi)*math.sin(psi), 
                        math.sin(phi)*math.sin(theta)*math.sin(psi)+math.cos(phi)*math.cos(psi), 
                        math.sin(phi)*math.cos(theta)],
                        [math.cos(phi)*math.sin(theta)*math.cos(psi)+math.sin(phi)*math.sin(psi),
                        math.cos(phi)*math.sin(theta)*math.sin(psi)-math.sin(phi)*math.cos(psi), 
                        math.cos(phi)*math.cos(theta)]])
        VECTOR = np.dot(ROTATION, np.array(HEADING))
        return VECTOR

    def inertialToBodyAngular(self, ATTITUDE, HEADING):
        phi = ATTITUDE[0]
        theta = ATTITUDE[1]

        ROTATION = np.array([[1, 0, -math.sin(theta)],
                        [0, math.cos(phi), math.sin(phi)*math.cos(theta)],
                        [0, -math.sin(phi), math.cos(phi)*math.cos(theta)]])

        VECTOR = np.dot(ROTATION, np.array(HEADING))
        return VECTOR