import numpy as np
import math

class MomentumTheorySolver():
    def __init__(self):
        pass        
    
    def forces( self, power, U):
        pass

class StripTheorySolver():
    def __init__(self, wing, liftfunction, elements):        
        self.wing = wing
        self.liftfunction = liftfunction
        self.elements = elements

        # calculation of constants
        self.el = wing.halfspan/elements
        self.CG = np.tile(self.wing.vehicle.cg,(self.elements,1))
        self.WINGNORM = np.tile(self.wing.NORM,(self.elements,1))
        self.K1 = self.wing.AR*self.wing.e*np.pi
        self.K2 = 0.5*self.wing.vehicle.environment.rho
        self.buildElementPositions()
        self.buildElementChords()
        self.dA = self.el*self.CHORD

        self.dX = self.wing.vehicle.cg[0]-self.WING[:,0]
        self.dY = self.wing.vehicle.cg[1]-self.WING[:,1]
        self.dZ = self.wing.vehicle.cg[2]-self.WING[:,2]
        
        # conversion vectors for projecting onto body X,Y,Z using the dot product
        self.bodX = np.tile([1,0,0],(self.elements,1))
        self.bodY = np.tile([0,1,0],(self.elements,1))
        self.bodZ = np.tile([0,0,1],(self.elements,1))

    def buildElementPositions(self):
        COORDS = self.wing.COORDS
        theta = self.wing.dihedral
        phi = self.wing.qtrchordsweep
        K = self.el/2*math.tan(phi)
        if COORDS[0][1]-COORDS[1][1] == 0:
            A = min(COORDS[0][2],COORDS[1][2])
            B = max(COORDS[0][2],COORDS[1][2])
            YS = np.linspace(COORDS[0][1], COORDS[1][1], self.elements)
            ZS = np.linspace(A+self.el/2,B-self.el/2,self.elements)
            if A == COORDS[0][2]:
                XS = np.linspace(COORDS[0][0]-0.25*self.wing.rc+K, COORDS[1][0]-0.25*self.wing.tc-K, self.elements)
            else:
                XS = np.linspace(COORDS[1][0]-0.25*self.wing.tc-K, COORDS[0][0]-0.25*self.wing.rc+K, self.elements)
        else:
            A = min(COORDS[0][1],COORDS[1][1])
            B = max(COORDS[0][1],COORDS[1][1])
            C = min(COORDS[0][2],COORDS[1][2])
            D = max(COORDS[0][2],COORDS[1][2])
            YS = np.linspace(A+self.el/2, B-self.el/2, self.elements)
            if A == COORDS[0][1]:
                XS = np.linspace(COORDS[0][0]-0.25*self.wing.rc+K, COORDS[1][0]-0.25*self.wing.tc-K, self.elements)
                if C == COORDS[0][2]:
                    ZS = np.linspace(COORDS[0][2]+self.el/2*math.tan(theta), COORDS[1][2]-self.el/2*math.tan(theta), self.elements)
                else:
                    ZS = np.linspace(COORDS[0][2]-self.el/2*math.tan(theta), COORDS[1][2]+self.el/2*math.tan(theta), self.elements)
            else:
                XS = np.linspace(COORDS[1][0]-0.25*self.wing.tc-K, COORDS[0][0]-0.25*self.wing.rc+K, self.elements)
                if C == COORDS[0][2]:
                    ZS = np.linspace(COORDS[1][2]-self.el/2*math.tan(theta), COORDS[0][2]+self.el/2*math.tan(theta), self.elements)
                else:
                    ZS = np.linspace(COORDS[1][2]+self.el/2*math.tan(theta), COORDS[0][2]-self.el/2*math.tan(theta), self.elements)    
        WING = []
        for i in range(0,self.elements):
            WING.append([XS[i],YS[i],ZS[i]])
        self.WING = np.asarray(WING)

    def plotElementPositions(self, ax):
        x = self.WING[:,0]
        y = self.WING[:,1]
        z = self.WING[:,2]
        ax.scatter(x,y,z,color='red')        
           
    def buildElementChords(self):
        COORDS = self.wing.COORDS
        theta = math.atan2(COORDS[1][2]-COORDS[0][2],COORDS[1][1]-COORDS[0][1])
        if COORDS[0][2]-COORDS[1][2] == 0:
            STATIONS = abs(self.WING[:,1])
        else:  
            STATIONS = abs(self.WING[:,1]/math.cos(theta))
        A = 2*self.wing.Sref/((1+self.wing.lmbd)*self.wing.halfspan)
        B = (1-self.wing.lmbd)/(self.wing.halfspan)
        self.CHORD = A*(1-B*STATIONS)
        
    def solveForces(self, U, P):
        V = U+np.cross(P, self.WING-self.CG)                                # calculate element velocities (body frame)
        projN = np.einsum("ij,ij->i",V,self.WINGNORM)                       # project velocity vector to wing norm axis
        projX = np.einsum("ij,ij->i",V,self.bodX)                           # project velocity vector to body X axis    
        PNsq = np.einsum("i,i->i",projN,projN)                              # squared magnitude of proj N
        PXsq = np.einsum("i,i->i",projX,projX)                              # squared magnitude of proj X
        Vsq = PNsq+PXsq                                                     # calculate squared magnitude of local V                                                   
        ALPHA = np.arctan2(projN,projX)                                     # calculate local angle of attack
        cl = self.liftfunction.CL(ALPHA)                                    # calculate lift coefficiencts using aoa
        cm = self.liftfunction.CM(ALPHA)                                    # calculate moment coefficients using aoa
        cd = cl**2/self.K1                                                  # calculate drag coefficients using cl
        LIFT = cl*self.K2*Vsq*self.dA                                       # calculate elemental lift vector
        DRAG = cd*self.K2*Vsq*self.dA                                       # calculate elemental drag vector
        PITCH = cm*self.K2*Vsq*self.dA*self.CHORD                           # calculate elemental pitching moment vector
        locXS = DRAG*np.cos(ALPHA)-LIFT*np.sin(ALPHA)                       # local X component
        locYS = np.zeros(np.size(locXS))                                    # local Y component (always zero)
        locZS = LIFT*np.cos(ALPHA)+DRAG*np.sin(ALPHA)                       # local Z component
        locF = np.asarray([locXS, locYS, locZS]).T                          # build local elemental force vector array
        projF = np.einsum("ij,ij->i",self.WINGNORM,locF)                    # project local force vector to wing norm
        F = np.einsum("ij,i->ij",self.WINGNORM,projF)                       # calculate wing norm force vector
        XS = np.einsum("ij,ij->i",self.bodX,locF)                           # project into body Xs for each element
        YS = np.einsum("ij,ij->i",self.bodY,F)                              # project into body Ys for each element
        ZS = np.einsum("ij,ij->i",self.bodZ,F)                              # project into body Zs for each element
        X, Y, Z = np.sum(XS), np.sum(YS), np.sum(ZS)                        # sum linear forces in body X, Y, Z
        L = np.sum(ZS*self.dY)+np.sum(YS*self.dZ)                           # sum moments about X axis (roll)
        M = -np.sum(ZS*self.dX)-np.sum(XS*self.dZ)                          # sum moments about Y axis (pitch)
        N = np.sum(XS*self.dY)+np.sum(YS*self.dX)                           # sum moments about Z axis (yaw)
        return np.asarray([[X, Y, Z], [L, M, N]])

    def plotLiftDistribution(self,ax):
        pass


class VortexLatticeSolver():
    def __init__(self):
        pass