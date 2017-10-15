import numpy as np

class IdealAirfoil():
    def __init__(self):
        pass

    def CL(self, alpha):
        res = 2*np.pi*alpha
        return res

    def CM(self,alpha):
        res = np.zeros(np.size(alpha))
        return res