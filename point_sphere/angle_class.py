import numpy as np

class angle:
    def __init__(self, angle_var, type):
        self.type = type
        self.angle_var = self.bound(angle_var)
    def __add__(self, other):
        return self.bound(self.angle_var+other)
    def bound(self, angle):
        if self.type == 'theta':
            return self.theta_bound(angle)
        elif self.type == 'phi':
            return self.phi_bound(angle)
    def theta_bound(self, theta):
        if theta >= 2*np.pi:
            theta = theta - 2*np.pi * int(theta/(2*np.pi))
        elif theta < 0:
            theta = theta + 2*np.pi * (1 + int(np.abs(theta)/(2*np.pi)))
        return theta
    def phi_bound(self, phi):
        if phi >= np.pi:
            phi = phi - np.pi * int(phi/(np.pi))
        elif phi < 0:
            phi = phi + np.pi * (1 + int(np.abs(phi)/(np.pi)))
        return phi