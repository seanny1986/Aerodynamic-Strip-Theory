import numpy as np
import math

class Wing():
    def __init__(self, vehicle, COORDS, e):
        self.vehicle = vehicle
        self.COORDS = np.asarray(COORDS)
        self.e = e
        
        # Calculations
        V1 = self.COORDS[0,0:3]-self.COORDS[1,0:3]
        V2 = self.COORDS[0,0:3]-self.COORDS[3,0:3]
        self.NORM = np.cross(V1,V2)/np.sqrt(np.sum(np.cross(V1,V2)**2))
        self.NORM[1], self.NORM[2] = -np.sign(V1[1])*self.NORM[1], np.abs(self.NORM[2])
        self.halfspan = np.sqrt(np.sum((self.COORDS[0]-self.COORDS[1])**2))
        self.rc = np.abs(self.COORDS[0][0]-self.COORDS[3][0])
        self.tc = np.abs(self.COORDS[1][0]-self.COORDS[2][0])
        self.cbar = (self.rc+self.tc)/2
        self.lmbd = self.tc/self.rc
        self.lesweep = math.atan2(self.COORDS[1][0]-self.COORDS[0][0],self.COORDS[1][1]-self.COORDS[0][1])
        X2 = self.COORDS[1][0]-0.25*self.tc
        X1 = self.COORDS[0][0]-0.25*self.rc
        self.qtrchordsweep = math.atan2(X2-X1,self.halfspan)
        self.Sref = self.halfspan*(self.rc+self.tc)/2
        self.AR = 4*self.halfspan**2/self.Sref
        self.dihedral = math.pi/2-math.acos(np.dot([0, np.sign(self.NORM[1])*1, 0],self.NORM))
        self.incidence = math.pi/2-math.acos(np.dot([1, 0, 0],self.NORM))

    def addSolver(self, solver):
        self.solver = solver

    def plotWing(self,ax):
        X1 = [self.COORDS[0][:],self.COORDS[1][:]]
        X2 = [self.COORDS[1][:],self.COORDS[2][:]]
        X3 = [self.COORDS[2][:],self.COORDS[3][:]]
        X4 = [self.COORDS[3][:],self.COORDS[0][:]]
        LIST = [X1,X2,X3,X4]
        for L in LIST:
            x = np.asarray(L)[:,0]
            y = np.asarray(L)[:,1]
            z = np.asarray(L)[:,2]
            ax.plot(x,y,z,color='black')

class Body():
    def __init__(self, vehicle, DIM, mass, cg):
        self.DIM = DIM
        self.mass = mass
        self.cg = cg
    
    def addSolver(self, solver):
        self.solver = solver
    
    def draw(self):
        pass

class Engine():
    def __init__(self, vehicle, diskRadius, POS, NORMAL):
        self.diskRadius = diskRadius
        self.POS = POS
        self.NORMAL = NORMAL
    
    def addSolver(self, solver):
        self.solver = solver

class Vehicle():
    def __init__(self, env, POSi, ATTi, Vb, Ab, Wb, Qb, cg, I):
        self.wings = []
        self.bodies = []
        self.engines = []
        self.bodyForces = np.asarray([0.,0.,0.])
        self.bodyMoments = np.asarray([0.,0.,0.])

        self.environment = env
        self.inertialPosition = np.asarray(POSi)
        self.inertialAttitude = np.asarray(ATTi)
        self.bodyLinearVelocity = np.asarray(Vb)
        self.bodyLinearAcceleration = np.asarray(Ab)
        self.bodyAngularVelocity = np.asarray(Wb)
        self.bodyAngularAcceleration = np.asarray(Qb)
        self.cg = np.asarray(cg)
        self.I = np.asarray(I)
    
    def plotComponents(self, ax):
        for w in self.wings:
            w.plotWing(ax)
            w.solver.plotElementPositions(ax)
        
        for e in self.engines:
            pass

        for b in self.bodies:
            pass 

    def addLinearVelocity(self, V):
        self.bodyLinearVelocity += V

    def addLinerAcceleration(self, A):
        self.bodyLinearAcceleration += A

    def addAngularVelocity(self, W):
        self.bodyAngularVelocity += W
    
    def addAngularAcceleration(self, Q):
        self.bodyAngularAcceleration += Q

    def addBodyForce(self, F):
        self.bodyForces += F

    def addMoment(self, M):
        self.bodyMoments += M

    def addWing(self, wing):
        self.wings.append(wing)

    def addBody(self, body):
        self.bodies.append(body)
        self.mass += body.mass

    def addEngine(self, engine):
        self.engines.append(engine)

    def forces(self):
        for eng in self.engines:
            F = eng.solver.solveForces()
            self.addBodyForce(F[0,:])
            self.addMoment(F[1,:])
        
        for w in self.wings:
            F = w.solver.solveForces(self.bodyLinearVelocity, self.bodyAngularVelocity)
            self.addBodyForce(F[0,:])
            self.addMoment(F[1,:])

        for b in self.bodies:
            F = b.solver.solveForces()
            self.addBodyForce(F[0,:])
            self.addMoment(F[1,:])
        
        print(self.bodyForces)
        print(self.bodyMoments)
         
    def stateupdate(self):
        A = self.bodyForces/self.mass
        B = self.environment.inertialToBodyDirectional(self.ATTITUDE,self.environment.G)
        C = -np.cross(self.addAngularVelocity,self.bodyLinearVelocity)
        dU_dt = A+B+C
        A = self.bodyMoments
        B = -np.cross(self.bodyAngularVelocity,np.matmul(self.I,self.bodyAngularVelocity))
        dW_dt = np.matmul(np.invert(self.I),A+B)
        self.bodyLinearVelocity += dVb_dt*self.env.dt
        self.bodyAngularVelocity += dWb_dt*self.env.dt
        self.inertialPosition += self.bodyToInertialDirectional(-self.inertialAttitude, self.bodyLinearVelocity)*self.environment.dt
        self.inertialAttitude += self.bodyToInertialAngular(-self.inertialAttitude, self.bodyAngularVelocity)*self.environment.dt

    # calculate body force and moment coefficients
    def coefficients(self):
        Sref = self.wings[0].Sref+self.wings[1].Sref
        span = self.wings[0].halfspan+self.wings[1].halfspan
        cbar = (self.wings[0].cbar+self.wings[1].cbar)/2
        q = 0.5*self.environment.rho*np.sum(self.bodyLinearVelocity**2)
        CX = self.bodyForces/q/Sref
        CL = self.bodyMoments[0]/q/Sref/span
        CM = self.bodyMoments[1]/q/cbar
        CN = self.bodyMoments[2]/q/Sref/span

        print(np.asarray([[CX[0], CX[1], CX[2]], [CL, CM, CN]]))

        return np.asarray([[CX[0], CX[1], CX[2]], [CL, CM, CN]])

    def bodyToInertialDirectional( self, ATTITUDE, DIRECTION):
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
        VECTOR = np.dot(np.transpose(ROTATION), np.array(DIRECTION))
        return VECTOR

    def bodyToInertialAngular(self, ATTITUDE, DIRECTION):
        phi = ATTITUDE[0]
        theta = ATTITUDE[1]

        ROTATION = np.array([[1, math.sin(phi)*math.tan(theta), math.cos(phi)*math.tan(theta)],
                        [0, math.cos(phi), -math.sin(phi)],
                        [0, math.sin(phi)/math.cos(theta), math.cos(phi)/math.cos(theta)]])

        VECTOR = np.dot(ROTATION, np.array(DIRECTION))
        return VECTOR