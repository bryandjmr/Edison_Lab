from point_class import point_s
from partition_class import partition
from visual_class import visual
from distance import geodesic_distance
import numpy as np

#this class houses the point on the sphere, and runs the simulation of points moving on its surface
class sphere:
    def __init__(self): #contains properties of a list of all the points and a data structure of the tessilationi of the surface of the sphere
        self.points = [] #list of all points on the surface of the sphere
        self.part = None #placeholder for plane list
        
    def add_p(self, position, velocity, radius): #adding a point to the sphere
        point = point_s(np.array(position), np.array(velocity), radius) #creates the point
        if point not in self.points:
            self.points.append(point) #adds it to point list
            if len(self.points) == 1: #once a point exists on the sphere, the  planes can then cover the sphere
                self.part = partition(point.omega) #creating the partition class
                self.part.tessellation() #dividing up the sphere based off of the points

            check = self.part.insert_p(point) #checking if point can be added without issue
            if check == False: 
                self.points.remove(point)
        else:
            print(point.p_s)
            print('ERROR: Point alreay exists on the Sphere', '\n')
    

    
    def simulation(self, time): #simulation function that prints a list of the times of the collosion and the points' position and creates video of collision
        if len(self.points) == 0: #determines that points have been added to sphere first
            print('ERROR: There are no points on the sphere', '\n')
        else:
            position_list= [0]*len(self.points) #creates empty array of the points' cartesian location
            video = visual(self.points[0].r) #creates data structure of video
            tracker = geodesic_distance(self.points[0].omega) #creates a data structure to graph distance between points

            #position_list = [0]*len(self.points)
            for i in range(len(self.points)): #finds initial location of the points
                #position_list[i] = self.points[i].p_s
                position_list[i] = self.points[i].conversion()

            video.add_images(position_list) #creating an image in the video data strcture

            tf = 0
            tracker.geodesic(self.points[0], self.points[1], tf)

            t_list = [0]*len(self.points)
            info_list = [0]*len(self.points)
            #prints out simulation data
            print('Information:', 'Simulation has started','\n''Time:', tf,'\n','Position:','\n',np.array(position_list),'\n')

            while tf <= time: #while loop runs until the end of simulation
                if tf == 0: #setting up the inital information of the points (critical time and critical events)
                    for i in range(len(self.points)):
                        t, info = self.points[i].p_plane.neighbors(self.points[i])
                        t_list[i] = t
                        info_list[i] = info

                #determine the minimum critical and event corresponding to it
                dt = min(t_list)
                min_index = t_list.index(dt)
                corr_info = info_list[min_index]

                if type(corr_info) == list: #if event is intersection get a list of all points involved in intersection
                    corr_info.append(self.points[min_index])
                    index_list = [self.points.index(point) for point in corr_info]
                    point_interest = self.points[min(index_list)] #point with lowest index involved in intersection
                    corr_info.remove(point_interest) #so point of interest doesn't compare itself to itself


                for i in range(len(self.points)): #pushes points foward then collects their position
                    if type(corr_info) == list:
                        if self.points[i] == point_interest:
                            #position_list[i] = self.points[i].step_forward(dt, corr_info, dt)
                            position_list[i] = self.points[i].step_forward(dt, corr_info, dt)
                        elif self.points[i] not in corr_info:
                            #position_list[i] = self.points[i].step_forward(t_list[i], info_list[i], dt)
                            position_list[i] = self.points[i].step_forward(t_list[i], corr_info, dt)
                        else:
                            #position_list[i] = self.points[i].step_forward(t_list[i], info_list[i], 0)
                            position_list[i] = self.points[i].step_forward(t_list[i], info_list[i], 0)
                    else:
                        #position_list[i] = self.points[i].step_forward(t_list[i], info_list[i], dt)
                        position_list[i] = self.points[i].step_forward(t_list[i], info_list[i], dt)

                video.add_images(position_list) #adds image of the current position to video

                tf += dt #updates time
                tracker.geodesic(self.points[0], self.points[1], tf) #finds current distance between points

                if dt != 0: #display location at nonzero time steps
                    print('Time:',round(tf, 8),'\n''Position:','\n',np.array(position_list),'\n')

                t_list = list(np.array(t_list) - dt) #updates t_list
                #finds out what points are involved in critical event so that its critical times and events can be updated
                if type(corr_info) == list:
                    special_points = corr_info
                    special_points.append(point_interest)
                else:
                    special_points = [self.points[t_list.index(time_step)] for time_step in t_list if time_step == 0]
                for i in special_points: #recalculating the critical time and critical event 
                    t, info = i.p_plane.neighbors(i)
                    corr_index = self.points.index(i)
                    t_list[corr_index] = t
                    info_list[corr_index] = info
                
            video.create_video() #deletes all the photos and creates a video
            tracker.graph() #creates a graph showing the distance between two points