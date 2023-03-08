from plane_class import plane
import numpy as np

class rectangle(plane): #subclass of plane, has a specific boundary function since it is a box
    def boundary(self, point):
        t_phi = 0
        t_theta = 0
        if point.v_s[1] != 0: #check if there is a phi component to the velocity
            if ((self.phi[1] - point.p_s[1])/point.v_s[1]) > 0: #checks to see if the point touches the top range of the phi boundary
                t_phi = ((self.phi[1] - point.p_s[1])/point.v_s[1])
            else: #if not the point actually intersects at the lower phi boundary
                t_phi = ((self.phi[0] - point.p_s[1])/point.v_s[1])
        if point.v_s[0] != 0: #similar setup to if conditions above but done for theta values and boundary
            if ((self.theta[1] - point.p_s[0])/point.v_s[0]) > 0:
                t_theta = ((self.theta[1] - point.p_s[0])/point.v_s[0])
            else:
                t_theta = ((self.theta[0] - point.p_s[0])/point.v_s[0])
        
        if ((t_phi == 0) and (t_theta == 0)) and ((point.v_s[0] != 0) or (point.v_s[1] != 0)): 
            return 0
        elif t_phi == 0 and t_theta == 0: #point does not escape
            t = float('inf')
        elif (t_theta == 0) and (point.v_s[0] == 0): #does not escape the theta boundaries
            t = t_phi
        elif (t_phi == 0) and (point.v_s[1] == 0): #does not escape the phi boundaries
            t = t_theta
        else: #sees which boundary the point will cross first
            if t_theta < t_phi:
                t = t_theta
            else:
                t = t_phi
        return t #returning time it exits