import numpy as np
from point_class import point_s

class plane: 
    def __init__(self, theta, phi, omega=0):
        self.theta = theta #theta range of plane
        self.phi = phi #phi range of plane
        self.p = [] #mulitple point can be placed in the plane but that means they overlap
        self.child = [] #list of all the neighbors of the plane
        self.omega = omega

    def __eq__(self, other): #defining equality of a plane compared to a list and other planes
        if type(other) == list:
            return False
        elif issubclass(type(other), plane) == True:        
            if self.theta == other.theta:
                if self.phi == other.phi:
                    return True
            return False

    def inside(self, point): #determines if a point is inside a plane
        if point.p_s[0] >= self.theta[0]:
            if point.p_s[0] <= self.theta[1]:
                if point.p_s[1] >= self.phi[0]:
                    if point.p_s[1] <= self.phi[1]:
                        if self.overlap_f(point) == True:
                            point.p_plane = self
                            self.p.append(point)
                            return [True, False]
                        else:
                            return False, True
        
        return False, False
        
    def overlap_f(self, point): #this function determines if there is an overlap with points in a plane or its neighbors
        if len(self.p) != 0: # testing overlap inside plane
            for i in self.p:
                overlap = i.overlapping(point)
                if overlap == True:
                    return False

        for i in self.child: #testing overlap for neighboring points
            if len(i.p) != 0:
                for j in i.p:
                    overlap = j.overlapping(point)
                    if overlap == True:
                        return False
        return(True)
        
    def add_child(self, grid): #adds neighbors to the plane
        if type(grid) == np.ndarray or type(grid) == list: #if plane is a list of planes then add all of them as neighbors
            for i in grid:
                if i not in self.child:
                    self.child.append(i)
                    i.child.append(self) #makes self a neighbor to other
        else: #adding a plane as neighbor if only one plane is given
            if self != grid:
                if grid not in self.child:
                    self.child.append(grid)
                    grid.child.append(self) 


    def neighbors(self, point): #checks the child list to see if any neighbors have a point and if they do, compute the intersection of the points
        t_list = []
        point_list = []
        for i in self.child:
            if len(i.p) > 0: #checking that neighbor has a point and then calculating intersection time
                for j in i.p:
                    t = point.inter_ps(j)
                    if t != float('inf'):
                        t_list.append(t) #appending time of intersection and neighbor
                        point_list.append(j)

        boundary = self.boundary(point)

        if len(t_list) != 0: #intersction times
            #finding the minimum time of intersection and what points are involved
            min_t = min(t_list)
            t_list_min = [time for time in t_list if time == min_t]
            min_index = [t_list.index(time) for time in t_list_min]
            info = [point_list[i] for i in min_index]
            if (min_t <= boundary) or (boundary < 0): #seeing if intersection is possible
                 return ([min_t, info])
            else: #intersections will not occur in under one revolution so do the boundary function
                return(boundary, 'Boundary')
        else: #no points are in the neighbors so just do the boundary function
            return(boundary, 'Boundary')


    def boundary_collision(self, p_1, p_2, t): #updates the position and velocity of both points participating in the colllision and feed those two points to the collision fixup function
        p_1.p_plane.p.remove(p_1)
        p_1.p_plane = None
        p_2.p_plane.p.remove(p_2)
        p_2.p_plane = None

        p_1.collision_update(p_2, t)
        self.collision_fixup(p_1) #feeds the point into collision fixup to assign it to a point
        self.collision_fixup(p_2)




    #need to figure out how to deal with n-body collision (please finish dealing with bugs first)
    def nboundary_collision(self, point, point_list, t):
        pass





    def collision_fixup(self, point, info=0): #sees if point is in current plane or one of its neighbors
        if self.inside(point) == True:
            point.p_plane = self
            self.p.append(point)
        else:
            self.update_boundary(point, info)


    
    def update_boundary(self, point, step_forward=0): #this function helps determine in which plane the point will end up in
        #have to work on this
        if step_forward == 1:
            self.p.remove(point)
            point.p_plane = None
        inside = False

        if (round(point.p_s[1], 8) == round(self.omega, 8)) and (round(2*np.pi, 8) != round(self.theta[1]- self.theta[0], 8)):
            if point.v_s[1] < 0: 
                for i in self.child:
                    if round(2*np.pi, 8) == round(self.theta[1]- self.theta[0], 8):
                        point.p_plane = i
                        i.p.append(point)
                        inside = True
                        break
        elif (round(point.p_s[1], 8) == round((np.pi-self.omega), 8)) and (round(2*np.pi, 8) != round(self.theta[1]- self.theta[0], 8)):
            if point.v_s[1] > 0:
                for i in self.child:
                    if round(2*np.pi, 8) == round(self.theta[1]- self.theta[0], 8):
                        point.p_plane = i
                        i.filled = True
                        i.p = point
                        inside = True
                        break
        if inside == False:
            if point.v_s[0] != 0:
                if point.v_s[0] > 0:
                    if round(point.p_s[0], 8) == round(2*np.pi, 8):
                        point.p_s[0] = 0
                    for i in self.child:
                            if point.p_s[0] >= i.theta[0]:
                                if point.p_s[0] < i.theta[1]:
                                    if point.p_s[1] >= i.phi[0]:
                                        if point.p_s[1] <= i.phi[1]:
                                            point.p_plane = i
                                            i.p.append(point)
                                            inside = True
                                            break
                else:
                    if point.p_s[0] == 0:
                        point.p_s[0] = 2*np.pi
                    for i in self.child:
                            if point.p_s[0] > i.theta[0]:
                                if point.p_s[0] <= i.theta[1]:
                                    if point.p_s[1] >= i.phi[0]:
                                        if point.p_s[1] <= i.phi[1]:
                                            point.p_plane = i
                                            i.p.append(point)
                                            inside = True
                                            break
        if inside == False:
            if point.v_s[1] != 0:
                if point.v_s[1] > 0:
                    for i in self.child:
                            if point.p_s[0] >= i.theta[0]:
                                if point.p_s[0] <= i.theta[1]:
                                    if point.p_s[1] >= i.phi[0]:
                                        if point.p_s[1] < i.phi[1]:
                                            point.p_plane = i
                                            i.p.append(point)
                                            inside = True
                                            break
                else:
                    for i in self.child:
                            if point.p_s[0] >= i.theta[0]:
                                if point.p_s[0] <= i.theta[1]:
                                    if point.p_s[1] > i.phi[0]:
                                        if point.p_s[1] <= i.phi[1]:
                                            point.p_plane = i
                                            i.p.append(point)
                                            inside = True
                                            break
        if inside == False:
            for i in self.child:
                    if point.p_s[0] >= i.theta[0]:
                        if point.p_s[0] <= i.theta[1]:
                            if point.p_s[1] >= i.phi[0]:
                                if point.p_s[1] <= i.phi[1]:
                                    point.p_plane = i
                                    i.p.append(point)
                                    inside = True
                                    break

        if inside == False:
            print('ERROR: Point has not been properly added to a plane')
            print('Point:', list(point.p_s), point.v_s, 'Original Plane:', self.theta, self.phi,'\n')
