# -*- coding: utf-8 -*-
"""
Created on Wed Apr 19 16:48:35 2023

@author: Christoph Schmidt
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.path as pth
from matplotlib.patches import PathPatch
from scipy import interpolate

from cyclistsocialforce.trajectory import generateSplinePrototype
from cyclistsocialforce.utils import angleDifference, angleSFMtoSUMO, limitAngle


#Imports that are only needed if the model ist used with SUMO
import traci



class SocialForceIntersection:
    
    def __init__(self, vehicleList, animate=False, axes=None, use_traci=False, 
                 insId="", node=[]):
        self.use_traci = use_traci
        self.vehicles = vehicleList
        self.nvec = len(vehicleList)
        
        self.vehicleX = np.zeros((self.nvec,1))
        self.vehicleY = np.zeros((self.nvec,1))
        self.vehicleTheta = np.zeros((self.nvec,1))
        self.updateVehiclePositions()
        
        self.animate = animate
        
        assert isinstance(insId, str), "Intersection ID has to be a string."
        self.id = insId
            
        # SUMO 
        #fig, ax = plt.subplots()
        
        if self.use_traci:
            
            #Intersection footprint 
            self.shape = pth.Path(node.getShape(), 
                                  closed=True)
            #ax.add_patch(PathPatch(self.shape))
            x,y = node.getCoord()
            
            #temp = np.array(node.getShape())
            #ax.plot(temp[:,0], temp[:,1], color="black")
            
            #Incoming and outgoing edges
            self.inEdges = {}
            self.outEdges = {}
        
            for e in node.getIncoming():
                lanes = e.getLanes()
                self.inEdges[e.getID()] = []
                for l in lanes:
                    #identify the closest point to the node center                  
                    path = np.array(l.getShape())
                    
                    tck,u = interpolate.splprep((path[:,0],path[:,1]),s=0.0, k=min(3,np.shape(path)[0]-1))
                    x_i,y_i= interpolate.splev(np.linspace(0,1,10),tck)
                    
                    x_i = x_i[-2:]
                    y_i = y_i[-2:]
                    
                    #ax.plot(x_i, y_i, color="green")
                    self.inEdges[e.getID()].append((x_i,y_i))

            
            for e in node.getOutgoing():
                lanes = e.getLanes()
                self.outEdges[e.getID()] = []
                for l in lanes:
                    #identify the closest point to the node center                  
                    path = np.array(l.getShape())
                    
                    tck,u = interpolate.splprep((path[:,0],path[:,1]),s=0.0, k=min(3,np.shape(path)[0]-1))
                    x_i,y_i= interpolate.splev(np.linspace(0,1,10),tck)
                    
                    x_i = x_i[:2]
                    y_i = y_i[:2]
                    
                    #ax.plot(x_i, y_i, color="red")
                    self.outEdges[e.getID()].append((x_i,y_i))
                    
        if animate:
            assert axes is not None, "Provide axes for animation!"
            self.ghandles = []
            #self.thandles = []
            #self.fhandles = []
            self.ax = axes
            self.prepareAxes()
            if self.use_traci:
                patch = PathPatch(self.shape, facecolor='black', edgecolor='black')
                self.ax.add_patch(patch)
    
            
    def addRoadUser(self, user):
        """Add a single road user to the current intersection.
        
        If the road user has route following activated and the simulation 
        operates with SUMO, this function also determines the trajectory
        protoype for the user. 
        
        Parameters
        ----------
        user : Vehicle
            Vehicle object to be added to this intersection
        """
        
        #determin destinations according to vehicle route
        if self.use_traci & user.follow_route:
            ecurrent = user.route[0]
            enext = user.route[1]
            
            assert ecurrent in self.inEdges, f"Road user {user.id} arriving \
                on junction {self.id} from unknown edge {ecurrent}!"
            assert enext in self.outEdges, f"Road user {user.id} requesting \
                to depart junction {self.id} on unknown edge {enext}!"
            
            #determine closest incoming lane
            lanepoints = self.inEdges[ecurrent]
            xpoints = np.concatenate((lanepoints[0][0], lanepoints[1][0]))
            ypoints = np.concatenate((lanepoints[0][1], lanepoints[1][1]))
            d = np.sqrt((xpoints-user.s[0])**2 +
                        (ypoints-user.s[1])**2)
            laneid_in = int(np.argmin(d)/2)

            #randomly select outgoing lane
            laneid_out = np.random.randint(0, len(self.outEdges[enext]))                
            
            
            #generate spline prototype between selected lanes
            points = np.vstack((np.array(self.inEdges[ecurrent][laneid_in]).T,
                               np.array(self.outEdges[enext][laneid_out]).T)) 
    
            xp, yp = generateSplinePrototype(points[:,0], points[:,1],15)
            
            #remove points behind the road user
            dp2f = np.sqrt((xp-xp[-1])**2 + \
                           (yp-yp[-1])**2)
            du2f = np.sqrt((user.s[0]-xp[-1])**2 + \
                           (user.s[1]-yp[-1])**2)
            xp = xp[dp2f<du2f]
            yp = yp[dp2f<du2f]
            
            #set destinations
            user.setDestinations(xp, yp, reset=True)
            
        
        #add road user to intersection
        self.vehicles.append(user)
        if self.nvec > 0:
            self.vehicleX = np.vstack((self.vehicleX, (user.s[0],)))
            self.vehicleY = np.vstack((self.vehicleY, (user.s[1],)))
            self.vehicleTheta =  \
                np.vstack((self.vehicleTheta, (user.s[2],)))
        else:
            self.vehicleX = np.array(((user.s[0],),))
            self.vehicleY = np.array(((user.s[1],),))
            self.vehicleTheta = np.array(((user.s[2],),))
        self.nvec += 1
        
    def getRoadUserIDs(self):    
        ruids = []
        for u in self.vehicles:
            ruids.append(u.id)
        return ruids
    
    def hasRoadUser(self, userId):
        """Check if a user with given ID is simulated by this intersection.
        
        Finds the first road user if multiple RUs with the same name are
        present. 
        
        Parameters
        ----------
        userId : str
            User ID given by TraCI.
        """
        assert isinstance(userId, str), "User ID has to be a string."
        
        for u in self.vehicles:
            if u.id == userId:
                return True
            
        return False
    
    def removeExited(self):
        """Remove road users that have exited the intersection from the list of
        road users
        """
        
        assert self.use_traci, "areOnIntersection() is meant for maintaining \
            an automatic vehicle list when using C-SFM with SUMO. For this, \
            the TraCI interface has to be activated!"
        
        offIntersection = np.logical_not(
                             self.areOnIntersection(np.c_[self.vehicleX, 
                                                          self.vehicleY]))
        if np.any(offIntersection):      
            self.vehicles = [self.vehicles[i] for i in range(len(self.vehicles)) if not offIntersection[i]]
            self.nvec = self.nvec-np.sum(offIntersection)
            
            offIntersection = np.argwhere(offIntersection).flatten()
            self.vehicleX = np.delete(self.vehicleX, offIntersection, 0)
            self.vehicleY = np.delete(self.vehicleY, offIntersection, 0)
            self.vehicleTheta = np.delete(self.vehicleTheta, offIntersection, 0)
            
        if self.nvec == 0 and self.animate:
            while len(self.ghandles) > 0:
                self.ghandles.pop(0).remove()
            
        
    def areOnIntersection(self, points):
        """Check if points are on the intersection footprint
        """
        
        assert self.use_traci, "areOnIntersection() is meant for maintaining \
            an automatic vehicle list when using C-SFM with SUMO. For this, \
            the TraCI interface has to be activated!"
            
        return self.shape.contains_points(points)           
        
            
    def prepareAxes(self):
        plt.sca(self.ax)
        if self.use_traci:
            plt.xlim([np.amin(self.shape.vertices[:,0])-1, 
                      np.amax(self.shape.vertices[:,0])+1])
            plt.ylim([np.amin(self.shape.vertices[:,1])-1, 
                      np.amax(self.shape.vertices[:,1])+1])
        else:
            plt.xlim([0, 12])
            plt.ylim([0, 12])
        #plt.title('Social Force Simulation')
        #plt.xlabel(r'$\frac{\it{x}}{\mathrm{m}}$')
        #plt.ylabel(r'$\frac{\it{y}}{\mathrm{m}}$')
        self.ax.set_aspect('equal', adjustable='box')

                
    def updateVehiclePositions(self):
        for i in range(0,self.nvec):
            self.vehicleX[i,0] = self.vehicles[i].s[0]
            self.vehicleY[i,0] = self.vehicles[i].s[1]
            self.vehicleTheta[i,0] = self.vehicles[i].s[2]
            
            if self.use_traci:
                traci.vehicle.moveToXY(self.vehicles[i].id,
                                       "",
                                       -1,
                                       self.vehicles[i].s[0],
                                       self.vehicles[i].s[1],
                                       angle = angleSFMtoSUMO(self.vehicles[i].s[2]),
                                       keepRoute=6)
        
        
    def calcForces(self):   
        #maintain the right number of arrow handles for drawing
        if self.animate:
            while len(self.ghandles) < self.nvec**2:
                self.ghandles.append(self.ax.arrow(0, 0, 1, 1, 
                                                   head_width=.3, 
                                                   head_length=.4, 
                                                   linewidth=2, 
                                                   edgecolor = "slategray", 
                                                   facecolor="slategray", 
                                                   animated=True))

                
            while len(self.ghandles) > self.nvec**2:
                self.ghandles.pop(0).remove()

                
            a = 0
                
        
        Fx = np.zeros((self.nvec, self.nvec))
        Fy = np.zeros((self.nvec, self.nvec))
        
        Fdestx = np.zeros((self.nvec))
        Fdesty = np.zeros((self.nvec))
        
        # calc destination force and repulsive forces for every vehicle
        for i in range(0,self.nvec):
            
            Fdestx[i], Fdesty[i] = self.vehicles[i].calcDestinationForce()
            
            #draw destination force
            if self.animate:
                self.ghandles[a].set_data(x=self.vehicleX[i,0], 
                                          y=self.vehicleY[i,0], 
                                          dx=Fdestx[i]/2, 
                                          dy=Fdesty[i]/2)

                a += 1
                
            if self.nvec > 1:
                Fxi, Fyi = self.vehicles[i].calcRepulsiveForce(
                    np.delete(self.vehicleX,i), np.delete(self.vehicleY,i))
                
                Fx[i,:i] = Fxi[0:i]
                Fx[i,i+1:] = Fxi[i:]
                
                Fy[i,:i] = Fyi[0:i]
                Fy[i,i+1:] = Fyi[i:]
                
        if self.nvec > 1:                   
            #Filter forces by FOV
            dX = np.tile(self.vehicleX.T, self.nvec) - \
                 np.tile(self.vehicleX, (self.nvec,1))
            dY = np.tile(self.vehicleY.T, self.nvec) - \
                 np.tile(self.vehicleY, (self.nvec,1))
            
            Fangle = limitAngle(np.arctan2(dY,dX).T)
            Vangle = np.tile(self.vehicleTheta, self.nvec).T
            
            for i in range(self.nvec):
                for j in range(self.nvec):
                    Fangle[i,j] = angleDifference(Vangle[i,j], Fangle[i,j])
    
                    
                    if abs(Fangle[i,j]) > self.vehicles[i].FOV:
                       Fx[i,j] = 0 
                       Fy[i,j] = 0 
                       
                       #print("Left field of view!")
                    
                
                    #Draw Repulsive Forces                       
                    if self.animate and i!=j: 
                        self.ghandles[a].set_data(x=self.vehicleX[j,0], 
                                                  y=self.vehicleY[j,0], 
                                                  dx=Fx[i,j]/2, 
                                                  dy=Fy[i,j]/2)
                        self.ax.draw_artist(self.ghandles[a])

                        a += 1
                                 
            #Add repulsive forces of others and destination force            
            Fx = np.sum(Fx,axis=0) + Fdestx
            Fy = np.sum(Fy,axis=0) + Fdesty
        else:
            Fx = Fdestx
            Fy = Fdesty
        
        for i in range(0,self.nvec):
            self.vehicles[i].force = (Fx[i], Fy[i])
            self.vehicles[i].F.append(np.sqrt(Fx[i]**2 + Fy[i]**2))
            
        return Fx, Fy
    
    def step(self):
        
        if self.nvec > 0:
            
            Fx, Fy = self.calcForces()
            
            for i in range(0,self.nvec):
                a, dpsi = self.vehicles[i].control(Fx[i], Fy[i])            
                self.vehicles[i].move(a, dpsi)
                if self.animate:
                    if self.vehicles[i].hasDrawings[0]:
                        self.vehicles[i].updateBikeDrawing(self.ax)
                    else:
                        self.vehicles[i].makeBikeDrawing(self.ax, 
                                                         drawTrajectory=True,
                                                         drawNextDest=True,
                                                         drawDestQueue=True,
                                                         drawPastDest=True,
                                                         drawForce=True,
                                                         drawPotential=False,
                                                         drawName=True)                       

                
            
            self.updateVehiclePositions()

                
    def endAnimation(self):
        """ End animation of the intersection  
    
        Set the "animated" property of all graphic object to False to prevent
        them from disappearing once the animation ends. 

        Returns
        -------
        None.

        """  
        if self.animate:
            for g in self.ghandles:
                g.set_animated(False)
            for v in self.vehicles:
                v.endAnimation()
            self.animate = False
            