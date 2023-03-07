import numpy as np
from matplotlib import pyplot as plt

class geodesic_distance: #creates a graph of the distance between one point and another as a function of time
    def __init__(self, omega):
        self.omega = omega #critical angle
        self.tracking = [] #list for distance
        self.time = [] #list for time

    def geodesic(self, p1, p2, time): #finds the angular distance between the two poitns and sub by critical angle
        base = p1.conversion()
        comp = p2.conversion()
        distance = np.sqrt(sum((base-comp)**2))
        angle = np.arcsin(distance/(2))

        self.tracking.append(angle-self.omega)
        self.time.append(time)


    def graph(self): #creating the graph of the distances of the two points
        plt.scatter(self.time, self.tracking)
        plt.xlabel('Time')
        plt.ylabel('Angle - Critical Angle')
        plt.title('Tracking the Angle Distance Between Two points')
        plt.show()
