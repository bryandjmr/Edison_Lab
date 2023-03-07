import numpy as np
from angle_class import angle

class point_s: #this class defines a point on a sphere and the points attribute
    def __init__(self, position, velocity, r):
        self.p_s = np.array(position)#position of the center of point (sphereical)
        self.v_s = np.array(velocity) #velocity of the point (spherical)
        if self.p_s[1] == 0 and self.v_s[1] == 0:
            self.p_s[0] = 0
        self.p_plane = None
        self.r = r #fraction of radius of point compared to big sphere
        self.omega = np.arccos((2 - r**2)/2) #this is angle on the big sphere with the point where another point will make contact

    def __eq__(self, other): #sets how to define equality by also factoring in when theta angle is 2*pi
        #factoring in theta angle when it is 2*pi
        if round(self.p_s[0], 8) == round(2*np.pi, 8):
            self.p_s[0] = 0
        if round(other.p_s[0], 8) == round(2*np.pi, 8):
            other.p_s[0] = 0 
           
        #equality is set by the position  
        if list(self.p_s) == list(other.p_s):
            return True
        else:
            return False

    def inter_ps(self, other): #determines intersection time of two other points
        def function(self, t): #returns back the distance between the two points and the velocity at which they approach each other 
            #setting placeholders to make plugging our variables into the equation more simplified
            pf1 = self.p_s[1] + t*self.v_s[1]
            pf2 = other.p_s[1] + t*other.v_s[1]
            pf_theta = abs(self.p_s[0]-other.p_s[0]) +  t*(self.v_s[0]-other.v_s[0])
            #the distance and velocity function describing the space between two points plugged in a specified time
            distance = np.cos(pf1)*np.cos(pf2) + np.sin(pf1)*np.sin(pf2)*np.cos(pf_theta) - np.cos(2*self.omega)
            velocity = -self.v_s[1]*np.sin(pf1)*np.cos(pf2) - other.v_s[1]*np.cos(pf1)*np.sin(pf2) + self.v_s[1]*np.cos(pf1)*np.sin(pf2)*np.cos(pf_theta) + other.v_s[1]*np.sin(pf1)*np.cos(pf2)*np.cos(pf_theta) - (self.v_s[0]-other.v_s[0])*np.sin(pf1)*np.sin(pf2)*np.sin(pf_theta)
            return(distance,velocity)

        def newt_raph_method(self, t, steps=0, error=1e-6): #repeats the function method until the approximation of the function is equal to zero
            dis,vel = function(self, t)
            if abs(dis) < error:
                return t
            elif steps == 1000: #this is the max recursive depth so we assume there is no intersection
                return(-1)
            else:
                return(newt_raph_method(self, t - dis/vel, steps+1, error))

        #this determines if an intersection will even be possible
        angle = other.p_s - self.p_s 
        angle_velocity = self.v_s - other.v_s
        t_1, t_2 = 0, 0
        if angle_velocity[0] != 0:
            t_1 = angle[0] / angle_velocity[0]
        if angle_velocity[1] != 0:
            t_2 = angle[1] / angle_velocity[1]

        if t_1 >= 0 and t_2 >= 0:
            tf= newt_raph_method(self, 0)
            if tf < 0: #no intersection
                return -tf
            return tf
        else:
            return float('inf')


    def final_p(self, t): #determines position of a point after a specific time
        final_p = self.p_s + (self.v_s * t) #gives position of point but the point must be corrected so its bounded
        corr_2 = abs(final_p[1])//(np.pi)
        if final_p[1] < 0: #if the phi value is negative
            if corr_2 % 2 == 0:
                final_p[1] = -(final_p[1] + corr_2*np.pi)
                if (round(final_p[0], 8) == 0) or (round(final_p[0], 8) == round(2*np.pi, 8)):
                    final_p[0] = np.pi
                elif round(final_p[0], 8) == round(np.pi, 8):
                    final_p[0] = 0
                self.v_s[1] = -self.v_s[1]
            else:
                final_p[1] = (corr_2+1)*np.pi + final_p[1]
                
        elif final_p[1] > np.pi: #if the phi value is positive
            if corr_2 %2 == 0:
                final_p[1] = final_p[1] - corr_2*np.pi
            else:
                final_p[1] = (1+corr_2)*np.pi - final_p[1]
                if (round(final_p[0], 8) == 0) or (round(final_p[0], 8) == round(2*np.pi, 8)):
                    final_p[0] = np.pi
                elif round(final_p[0], 8) == round(np.pi, 8):
                    final_p[0] = 0
                self.v_s[1] = -self.v_s[1]
                
        corr_1 = abs(final_p[0])//(2*np.pi)
        
        
        if round(final_p[0], 8) >= 2*np.pi: #tracks when theta for position is bigger than 2pi and smaller than 0
            final_p[0] = final_p[0] - corr_1*2*np.pi
        elif round(final_p[0], 8) <= 0:
            final_p[0] = final_p[0] + (corr_1+1)*2*np.pi
        

        if final_p[1] == 0 and self.v_s[1] == 0:
            final_p[0] = 0

        self.p_s = final_p

    
    def collision_update(self, other, t): #calculates new starting position and velocity of points after a collision
        #setting new starting position
        self.final_p(t)
        other.final_p(t)
        base = self.v_s - other.v_s
        comp = other.v_s
        #setting new velocity after collision
        self.v_s = comp
        other.v_s = base + comp

        if (round(self.p_s[1], 8) == 0) or (round(self.p_s[1], 8) == round(np.pi, 8)):
            self.v_s[1] = -self.v_s[1]
            self.p_s[0] = other.p_s[0] + np.pi
        
        counter_1 = abs(self.p_s[0]) // (2*np.pi) 
        if round(self.p_s[0], 8) <= 0:
            self.p_s[0] = self.p_s[0] + (counter_1+1)*2*np.pi
        elif round(self.p_s[0], 8) >= 2*np.pi:
            self.p_s[0] = self.p_s[0] - counter_1*2*np.pi

        if round(other.p_s[1], 8) == 0 or round(other.p_s[1], 8) == round(np.pi):
            other.v_s[1] = -other.v_s[1]
            other.p_s[0] = self.p_s[0] + np.pi
    
        counter_2 = abs(other.p_s[0]) // (2*np.pi) 
        if round(other.p_s[0], 8) <= 0:
            other.p_s[0] = other.p_s[0] + (counter_2+1)*2*np.pi
        elif round(other.p_s[0], 8) >= 2*np.pi:
            other.p_s[0] = other.p_s[0] - counter_2*2*np.pi
        

    def cycle_t(self): #this function determines returns the time interval it takes for one point to make one revolution
        velocity = sum(self.v_s**2)
        if velocity == 0:
            return float('inf')
        else:
            return(2*np.pi / velocity)

    def conversion(self):
        x = np.cos(self.p_s[0])*np.sin(self.p_s[1])
        y = np.sin(self.p_s[0])*np.sin(self.p_s[1])
        z = np.cos(self.p_s[1])
        return np.array([x,y,z])
    
    def overlapping(self, other): #determines if there is an overlap with a currently existing point
        base = self.conversion()
        comp = other.conversion()
        distance = np.sqrt(sum((base-comp)**2))
        angle = np.arcsin(distance/2) #calculating the angle between two points
        
        if angle < self.omega: #testing if the angle is less than the critical angle
            return(True)
        else:
            return(False)

    def step_forward(self, t_critical, info, t_step): #pushes a point foward by a specific time 
        if t_step == t_critical: #sees if a point is at its critical time
            if type(info) == list: #is critical time for intersection
                if len(info) == 1: #1 body intersection
                    self.p_plane.boundary_collision(self, info[0], t_step)
                else: #n body intersection
                    self.p_plane.nboundary_collision(self, info, t_step)
            elif info == 'Boundary': #is critical time for boundary
                self.final_p(t_step+5e-15) #updates position of point added with error
                self.p_plane.update_boundary(self, 1)
        else: #point is just stepping foward in time (no critical moment)
            if t_step != 0:
                self.final_p(t_step)
        return self.conversion()
    