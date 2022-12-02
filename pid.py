from time import time
class Pid:
    def __init__(self):
        self.rD = 130
        self.eD = 0
        self.eD_ = 0
        self.eD__ = 0
        
        self.posD_ = 0.0
        self.posD__ = 0.0
        self.dt = 0.0
        self.t = 0
        self.t_ = 0
        self.t__ = 0

        self.kpD = 2
        self.kiD = 0.00001
        self.kdD = 0.001
        self.k0D = 0
        self.k1D = 0
        self.k2D = 0

    def pos_motor(self, pos):
        self.posD__ = self.posD_
        self.posD_ = pos

    def vel(self):
        return ((self.posD_ - self.posD__)/180)/self.dt
        
        pass

    def loop(self, A):
        
        self.t_ = self.t
        self.t = time()
        dt = self.t - self.t_
        vd = self.vel()
        if self.t - self.at__ > 10000:
            self.pD_ = self.pD
            self.pD = -self.posD

            dt = float(self.t - self.at__) / 1000000.0
            #velocidad(dt)

            outD_ = outD
            eD__ = eD_
            eD_ = eD
            eD = rD - vD

            #// Complete aquí

            k0D = self.kpD * (1 + dt * self.kiD + self.kdD / dt)
            k1D = -self.kpD * (1 + 2*self.kdD / dt)
            k2D = self.kpD * self.kdD / dt
            outD = outD_ + k0D * eD + k1D * eD_ + k2D * eD__