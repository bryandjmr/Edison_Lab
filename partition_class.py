from circle_class import circle
from rectangle_class import rectangle
import numpy as np

class partition:
    def __init__(self, omega): 
        self.planes = [] #list of all the planes
        self.omega = omega #angle of contact


    def create_theta(self, phi): #creates a numpy list of theta dependent on the phi value ring
        gamma = np.sin(phi)
        theta_var = 2.01*self.omega/gamma
        interval = int(np.ceil((np.pi*2)/theta_var))
        theta = np.linspace(0, 2*np.pi, interval)
        return(theta_var, theta) #returns the step size and an array of the breakup of the theta ring


    def comparison(self, rect, other, theta_var): #finding which planes are neighbors
        base = rect
        comp = other
        added = False
        if comp.theta[0] <= (base.theta[0] - 2*theta_var):
            if comp.theta[1] >= (base.theta[0] - 2*theta_var):
                if other not in rect.child:
                    added = True
                    rect.add_child(other)
        elif comp.theta[0] >= (base.theta[0] - 2*theta_var):
            if comp.theta[0] <= (base.theta[1] + 2*theta_var):
                if other not in rect.child:
                    added = True
                    rect.add_child(other)
        elif comp.theta[1] >= (base.theta[0] - 2*theta_var):
            if comp.theta[1] <= (base.theta[1] + 2*theta_var):
                if other not in rect.child:
                    added = True
                    rect.add_child(other)
        elif comp.theta[1] <= (base.theta[1] + 2*theta_var):
            if comp.theta[1] >= (base.theta[0] - 2*theta_var):
                if other not in rect.child:
                    added = True
                    rect.add_child(other)

            if added == False:
                if (rect.theta[0] - 2*theta_var) <= 0:
                    if comp.theta[0] <= (base.theta[0] - 2*theta_var + 2*np.pi):
                        if comp.theta[1] >= (base.theta[0] - 2*theta_var + 2*np.pi):
                            if other not in rect.child:
                                rect.add_child(other)
                    elif comp.theta[0] >= (base.theta[0] - 2*theta_var + 2*np.pi):
                        if comp.theta[0] <= (base.theta[1] + 2*theta_var + 2*np.pi):
                            if other not in rect.child:
                                rect.add_child(other)
                    elif comp.theta[1] >= (base.theta[0] - 2*theta_var + 2*np.pi):
                        if comp.theta[1] <= (base.theta[1] + 2*theta_var + 2*np.pi):
                            if other not in rect.child:
                                rect.add_child(other)
                    elif comp.theta[1] <= (base.theta[1] + 2*theta_var + 2*np.pi):
                        if comp.theta[1] >= (base.theta[0] - 2*theta_var + 2*np.pi):
                            if other not in rect.child:
                                rect.add_child(other)
                elif (rect.theta[1] + 2*theta_var) >= 2*np.pi:
                    if comp.theta[0] <= (base.theta[0] - 2*theta_var - 2*np.pi):
                        if comp.theta[1] >= (base.theta[0] - 2*theta_var - 2*np.pi):
                            if other not in rect.child:
                                rect.add_child(other)
                    elif comp.theta[0] >= (base.theta[0] - 2*theta_var - 2*np.pi):
                        if comp.theta[0] <= (base.theta[1] + 2*theta_var - 2*np.pi):
                            if other not in rect.child:
                                rect.add_child(other)
                    elif comp.theta[1] >= (base.theta[0] - 2*theta_var - 2*np.pi):
                        if comp.theta[1] <= (base.theta[1] + 2*theta_var - 2*np.pi):
                            if other not in rect.child:
                                rect.add_child(other)
                    elif comp.theta[1] <= (base.theta[1] + 2*theta_var - 2*np.pi):
                        if comp.theta[1] >= (base.theta[0] - 2*theta_var - 2*np.pi):
                            if other not in rect.child:
                                rect.add_child(other)
                
        
    def tessellation(self): #divides the surface of the sphere into semi equal portions
        self.planes.append(circle([0, np.pi*2], [0, self.omega], self.omega)) #make the area slightly smaller than self.area to ensure that only one point can exist inside
        phi = np.linspace(self.omega, np.pi-self.omega, int((np.pi-2*self.omega)/(2*self.omega)))
        prev_row = []
        for i in range(len(phi)): #loops through phi list
            if phi[i] != phi[-1]:
                theta_var, theta = self.create_theta(phi[i]) #helps solve for the interval that theta needs to be to be more than twice the size of a point
                neighbors = np.empty(len(theta)-1, dtype=rectangle)#this list will be populated with current phi row that is be created to then find the neighers in the phi row (neighbors that are side by side)
                for j in range(len(theta)):
                    if theta[j] != theta[-1]:
                        rect = rectangle([theta[j], theta[j+1]], [phi[i], phi[i+1]]) #creating the plane
                        neighbors[j] = rect
                        if j != 0:
                            rect.add_child(neighbors[j-1])
                            if theta[j] == theta[-2]:
                                rect.add_child(neighbors[0])

                        if i != 0: 
                            if j == 0 or theta[j] == theta[-2]:
                                rect.add_child([prev_row[0], prev_row[-1]])
                            for k in prev_row: #checks to see if the neighbors from above touch rect
                                self.comparison(neighbors[j], k, theta_var)
                    
                if i == 0: #adding the neighbors of the first phi row to the circle class above 
                    self.planes[0].add_child(neighbors)
                self.planes.append(neighbors) #adding each row as a list to the self.planes function
                prev_row = neighbors #is a list that holds the planes of the previous row so that neighbors can be added
                
            else:
                self.planes.append(circle([0,np.pi*2], [np.pi-self.omega, np.pi], self.omega))
                self.planes[-1].add_child(neighbors)

                          
    def insert_p(self, point): #adds a point to the specific plane it corresponds to
        done = False
        overlap_counter = False
        
        for i in self.planes:
            if done == True or overlap_counter == True:
                break
            if type(i) == circle: #looking at circles
                done,overlap_counter = i.inside(point) #gets truth values of if point has been inserted or point overlaps
                if done == True or overlap_counter == True:
                    break
            else: #looking at rectangles
                for rect in i:
                    done,overlap_counter = rect.inside(point)
                    if (done == True) or (overlap_counter == True):
                        break
                

        if overlap_counter == True: #returns error message if a point overlaps
            print(point.p_s)
            print('ERROR: Point cannot be placed because it overlaps with a pre-exisiting point', '\n')
            return False

        elif done == False: #returns error message if point outside bounds of the sphere
            print(point.p_s)
            print('ERROR: Values placed for position of the point are outsides the bounds of the surface of the sphere', '\n')
            return False
        else:
            return True