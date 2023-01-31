import numpy as np

class ReferenceTrajectory(object):
    def __init__(self, curve="spiral", ts=0.1, N=5000, space=[0.5,0.5,1.5], tscale=0.1):
        """Create object of reference trajetcory

        Args:
            curve (str, optional): figure8 or spiral. Defaults to "figure8".
            ts (float, optional): sampling period of reference trajectory. Defaults to 0.01.
            N (int, optional): number of samples. Defaults to 50000.
            space (float, optional): size of flight space in m. Defaults to 2.
        """
        self.t = np.cumsum(ts*np.ones(N))
        self.tscale = tscale
        if hasattr(space, "__len__"):
            self.space = space
        else:
            self.space = np.array([space, space, space])
        if hasattr(tscale, "__len__"):
            self.tscale = tscale
        else:
            self.tscale = np.array([tscale, tscale, tscale])
        self.curve = curve
        if self.curve == "figure8":
            self.figure8()
        else:
            self.spiral()

    def figure8(self):
        """Lemniscate of Gerono/Figure8 trajectory"""
        self.x = self.space[0]*np.cos(self.tscale[0]*self.t)
        self.y = self.space[1]*np.sin(self.tscale[1]*2*self.t) / 2
        self.z = self.space[2]*np.ones(len(self.t))


    def spiral(self):
        """Spiral trajectory"""
        self.x = self.space[0]*np.cos(self.tscale[0]*self.t)
        self.y = self.space[1]*np.sin(self.tscale[1]*self.t)
        self.z = self.space[2]*np.ones(len(self.t))
        
    def get_polynomial_traj(self):
        # # Define waypoints as an array
        # waypoints = np.array([[x1, y1, t1], [x2, y2, t2], [x3, y3, t3], ...])

        # Calculate polynomial coefficients
        coefficients_x = np.polyfit(self.t, self.x,7)
        coefficients_y = np.polyfit(self.t, self.y,7)
        coefficients_z = np.polyfit(self.t, self.z,7)
        
        trajectory = np.array([coefficients_x,coefficients_y,coefficients_z])
        
        return trajectory
    

if __name__ == '__main__':
    ref = ReferenceTrajectory("spiral", ts=0.1, N=5000, space=[0.5,0.5,1.5], tscale=0.1)