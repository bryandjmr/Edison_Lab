from plane_class import plane
import numpy as np

class circle(plane): #subclass of plane, has a specific boundary function since circle only exists at north and south pole
    def boundary(self, point): #defines the time it takes for a point to leave the circle plane
        t = float('inf')
        if point.v_s[1] != 0: #will always escape region as long as it has some velocity in the phi direction
            if self.phi[0] == 0: #checks to see what circle you are in (north or south)
                t = self.top_circle(point)
            elif round(self.phi[0], 8) == round(np.pi-self.omega, 8): #circle at south pole
                t = self.bottom_circle(point)
            
            if (t == 0) and (point.v_s[1] != 0):
                return 0
            
        return t

    def top_circle(self, point): #finds time for a point on the north pole
        movement = point.p_s[1] / point.v_s[1]
        if movement == 0:
            t = self.omega / abs(point.v_s[1])
        elif movement > 0:
            t = (self.omega - point.p_s[1]) / point.v_s[1]
        else:
            t = -(self.omega + point.p_s[1]) / point.v_s[1]
        return t
    
    def bottom_circle(self, point): #finds time for a point on the south pole
        movement = (np.pi - point.p_s[1]) / point.v_s[1]
        if movement == 0:
            t = self.omega / abs(point.v_s[1])
        elif movement > 0:
            t = (np.pi - point.p_s[1] + self.omega) / point.v_s[1]
        else:
            t = (np.pi - self.omega - point.p_s[1]) / point.v_s[1]
        return t