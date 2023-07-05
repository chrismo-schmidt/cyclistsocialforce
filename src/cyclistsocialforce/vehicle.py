# -*- coding: utf-8 -*-
"""
Created on Wed Apr 19 16:49:26 2023

@author: Christoph Schmidt
"""
import numpy as np

from matplotlib.patches import Ellipse
from scipy import interpolate

from cyclistsocialforce.utils import limitAngle, cart2polar, thresh, angleDifference, toDeg
from cyclistsocialforce.control import PIDcontroller


class Vehicle:
    
    #Constants 
    VMAX = (-1,7)
    AMAX = (-10,10)
    PSIMAX = (80/360)*(2*np.pi)
    STEERMAX = ((-60/360)*(2*np.pi),(60/360)*(2*np.pi))
    FOV = (80/360)*(2*np.pi)
    V0 = 5
    PDECAY = 2
    P0 = 30 #
    PMAX = 0.8
    FDEST0 = 6
    RELAX = .1
    DT = .01
    WHEELBASE = 1
    DARRIVED = 2    # dist. to current destination for switch to next dest.

    
    def __init__(self, s0, userId="unknown", route=()):
        """Create a Vehicle
        
        Creates a vehicle object designed to model the planar dynamics of a
        vehicle in 2D space and calculate corresponding social forces according
        to the Cyclist Social Force concept (CSF)
        
        If a route is provided, an intersection object that owns this 
        vehicle will assing destinations for crossing an intersection according
        to this route. 
        
        Definition of state variables:
            x: position [m] in the global coordinate frame
            y: position [m] in the global coordinate frame
            theta: heading [rad] relative to the positive x-axis in global 
                   frame. Positive angles go counterclockwise.
            v: longitudinal speed [m/s] of the vehicle 
            psi: steer angle relative to the heading. Positive angles go 
                 counterclockwise.
        
        
        Parameters
        ----------
        s0 : Tuple of float
            Tuple containing the initial vehicle state (x, y, theta, v, psi).
        userId : str, default = ""
            String for identifying this vehicle.
        route : list of str, default = []
            List of edge IDs describing the vehicle route. An empty list 
            deactivates route following and destinations have to be set 
            using vehicle.setDestinations()
        """
        
        # vehicle state (x, y, theta, v, psi)
        self.s = list(s0)
        self.s[2] = limitAngle(s0[2])
        self.updateExcentricity()
        
        # identification
        assert isinstance(userId, str), "User ID has to be a string."
        self.id = userId
        
        # follow a route
        assert isinstance(route, tuple), "Route has to be a tuple"
        assert all(isinstance(r, str) for r in route), "Edge IDs in route list have to be str"
        self.follow_route = bool(route)
        self.route = route
        
        # next destination and queue of following destination
        self.dest = s0[0:2]
        self.destqueue = np.c_[self.dest[0],self.dest[1]]
        self.destpointer = 0
        
        # trajectory (list of past states)
        self.traj = ([self.s[0]], 
                     [self.s[1]], 
                     [self.s[2]], 
                     [self.s[3]], 
                     [self.s[4]])
        
        # vehicle control inputs and controllers (separate controlers for speed
        # and steer angle)
        self.controlinput = ([],[])
        self.controlsignals = ([],[])
        self.controllers = (PIDcontroller(12, 0,0, self.DT, isangle=True), 
                            PIDcontroller(8, 0, 0, self.DT, isangle=False))
        # ego repulsive force
        self.F = []
        
        # has a drawing 
        self.hasDrawings = [False] * 8
        
    def isLastDest(self):
        if self.destqueue is None:
            return True
        if self.destpointer+1 >= np.shape(self.destqueue)[0]:
            return True
        return False
        
    def updateExcentricity(self):
        """Update the execentricity of the ego repulsive potential
        
        The potentials for repulsive force calculations are ellipses with 
        speed-depended excentricity. This updates the excentricity according
        the current speed. 
            
        """
        self.e = min(np.power(self.s[3]/self.VMAX[1], 0.1), 0.7) 
        
        
    def calcPotential(self, x, y):
        """Evaluate the repulsive potential of the ego vehicle. 
        
        Evaluates at all locations in x and y. 
        
        
        Parameters
        ----------
        x : Tuple of floats
            x locations for potential evaluation. len(x) must be equal len(y)
        y : Tuple of floats
            y locations for potential evaluation. len(x) must be equal len(y)
        
            
        """
        
        # coordinate transformations
        x0 = x - self.s[0] #+.5*np.cos(self.s[2])
        y0 = y - self.s[1] #-.5*np.sin(self.s[2])
        
        rho, phi = cart2polar(x0, y0)
        
        phi0 = angleDifference(phi, self.s[2])
        phi0 = phi - self.s[2]
        
        # excentricity of elliptic potential according to vehicle speed
        self.updateExcentricity()
                
        # elliptic potential
        b = (1/(np.sqrt(1-np.power(self.e,2))*self.PDECAY)) * rho * (1 - self.e * np.cos(phi0))
        
        P = self.P0 * np.exp(-b)
        #P[P>self.PMAX] = self.PMAX
        
        return P
    
    
    def calcRepulsiveForce(self, x, y):
        """Evaluate the repulsive force of the ego vehicle. 
        
        Evaluates at all locations in x and y. 
        
        
        Parameters
        ----------
        x : Tuple of floats
            x locations for potential evaluation. len(x) must be equal len(y)
        y : Tuple of floats
            y locations for potential evaluation. len(x) must be equal len(y)        
            
        """
        
        # coordinate transformations
        x0 = x - self.s[0]
        y0 = y - self.s[1]
        
        rho, phi = cart2polar(x0, y0)
        
        phi0 = angleDifference(phi, self.s[2])
        phi0 = phi - self.s[2]
        
        # calc the potential gradient
        P = self.calcPotential(x, y)/self.PDECAY
        
        # force
        Frho0 =  P * ((1 - self.e * np.cos(phi0)) / np.sqrt(1 - np.power(self.e,2)))
        Fphi0 =  P * ((self.e * np.sin(phi0)) / np.sqrt(1 - np.power(self.e,2)))
        
        # transform coordinates back
        
        Fx = Frho0 * np.cos(phi) - Fphi0 * np.sin(phi)
        Fy = Frho0 * np.sin(phi) + Fphi0 * np.cos(phi)
        
        return (Fx, Fy)
    
    def calcDestinationForceField(self, x, y):
        """Calculates force vectors from locations in x, y to the current 
        destination. 
        
        Evaluates at all locations in x and y. 
        
        
        Parameters
        ----------
        x : Tuple of floats
            x locations for potential evaluation. len(x) must be equal len(y)
        y : Tuple of floats
            y locations for potential evaluation. len(x) must be equal len(y)        
            
        """
        
        ddest = np.sqrt(np.power(self.dest[0]-x,2) + np.power(self.dest[1]-y,2))
        
        if ddest > 0:
            Fx = -self.FDEST0 * (x-self.dest[0]) / ddest
            Fy = -self.FDEST0 * (y-self.dest[1]) / ddest
        else:
            Fx = 0
            Fy = 0 
        
        return (Fx, Fy)
    
    def updateDestination(self):
        """
        Check if the road user has reached the current intermediate destination
        and update to the next destination. 

        """
        assert self.destqueue is not None, "Road user does not have a \
            destination queue!"
        
        #check if road user arrived at next destination
        dnext = self.getDestinationDistance()
        
        if dnext <= self.DARRIVED:
            self.destpointer = min(self.destpointer+1, 
                                   np.shape(self.destqueue)[0]-1)
              
        #check if we can jump the next destination in the queue 
        if self.destpointer < (np.shape(self.destqueue)[0]-1):
            dnextnext = np.sqrt((self.destqueue[self.destpointer+1,0]- \
                                 self.s[0])**2 + \
                                (self.destqueue[self.destpointer+1,1]-
                                 self.s[1])**2)
            if dnextnext < dnext:
                self.destpointer += 1
        
        # update destination 
        self.dest = self.destqueue[self.destpointer,:]        
    
    def calcDestinationForce(self):
        """Calculates force vectors from the current ego vehicle location to
        the current destination. 
        
        If the road user has a destination queue, this also checks if the
        current intermediate destination has to be updated to the next one.
        """
        if self.destqueue is not None:
            self.updateDestination()
        
        return self.calcDestinationForceField(self.s[0], self.s[1])
    
    def calcDestinationForceHM(self):
        """Calculates the destination force according to Helbing and Molnar
        
        """
        
        rx, ry = self.calcDestinationForce()
        
        r = np.sqrt(rx**2+ry**2)
        ex = rx/r
        ey = ry/r
        
        Fx = (1/self.RELAX)*(self.V0*ex - (self.s[3] * np.cos(self.s[2])))
        Fy = (1/self.RELAX)*(self.V0*ey - (self.s[3] * np.sin(self.s[2])))
        
        return (Fx, Fy)
    
    def getDestinationDistance(self):
        """Return the current distance of the ego vehicle to it's destination. 
        
        """
        return np.sqrt(np.power(self.dest[0]-self.s[0],2) + np.power(self.dest[1]-self.s[1],2))
    
    def control(self, Fx, Fy):
        """Calculate control effort based on the current social force 
        
        """
        
        ddest = self.getDestinationDistance() 
        
        theta = np.arctan2(Fy,Fx)
        v = np.sqrt(np.power(Fx,2)+np.power(Fy,2))
        
        ddest = np.sqrt(np.power(self.dest[0]-self.s[0],2) + np.power(self.dest[1]-self.s[1],2))

        if ddest < 3 and self.isLastDest():
            v = (v/3)*ddest
        self.controlinput[1].append(v)
        
        target_angle_ego = angleDifference(self.s[2], theta)

        self.controlinput[0].append(target_angle_ego)
        
        dpsi = angleDifference(self.s[4], target_angle_ego) 
        dv = v - self.s[3]
        
        opsi = self.controllers[0].step(dpsi)
        a = self.controllers[1].step(dv)        

        return a, opsi
    
    def move(self, a=0, dpsi=0):
        """Propagate the dynamic vehicle model by one step.
        
        """
        
        #vx = self.s[3] * np.cos(self.s[2])
        #vy = self.s[3] * np.sin(self.s[2])
        
        #ax = self.a[0]
        #ay = self.a[1]
        
        # update
        #x = self.s[0] + vx * self.DT + ax * self.DT**2
        #y = self.s[1] + vy * self.DT + ay * self.DT**2
        
        #vx = vx + ax * self.DT
        #vx = vx + ax * self.DT
        #theta = np.tan(self.a[1]/self.a[0])
    
        #s_new = self.s;
        
        # s = [x,y,theta,v,psi]
        a = thresh(a, self.AMAX)
        
        self.controlsignals[0].append(dpsi)
        self.controlsignals[1].append(a)
        
        psi = limitAngle(self.s[4] + self.DT * dpsi)
        v   = self.s[3] + self.DT * a
        
        psi = thresh(psi, (-self.PSIMAX, self.PSIMAX))
        v = thresh(v, self.VMAX)
        
        theta  =self.s[2] + self.DT * v * np.tan(psi) / self.WHEELBASE
        
        theta = limitAngle(theta)
        
        y = self.s[1] + self.DT * v * np.sin(theta)
        x = self.s[0] + self.DT * v * np.cos(theta)

        self.s = [x,y,theta,v,psi]
        
        self.traj[0].append(x)
        self.traj[1].append(y)
        self.traj[2].append(theta)
        self.traj[3].append(v)
        self.traj[4].append(psi)
            
        
    
    def setDestinations(self, x, y, reset=False):
        """Set the next destinations.
        
        Use this function to pass a trajectory prototype to the Cyclist-SFM.
        
        
        Parameters
        ----------
        x : List of float
            List of the next destination x coordinates. First destination has 
            to be at x[0].
        y : List of float
            List of the next destination x coordinates. First destination has 
            to be at y[0].
        reset : bool
            If false, the new destinations are appended to the end of the
            queue. If true, the current queue is deleted and the the first 
            element of dest becomes the immediate next destination.
        """
        
        if reset or self.destqueue is None:
            self.destqueue = np.c_[x,y]
            self.destpointer = 0
        else:
            self.destqueue = np.vstack((self.destqueue, np.c_[x,y]))
        
            
    def setSplineDestinations(self, x, y, npoints, reset=False):
        """Calculate and set intermediate destination according to 2d order
        """
        assert len(x)>=3, "Provide at least 3 points to calculate a cubic trajectory prototype"
        
        x = np.insert(np.array(x), 0, self.s[0])
        y = np.insert(np.array(y), 0, self.s[1])
        tck,u = interpolate.splprep((x,y),s=0.0)
        x_i,y_i= interpolate.splev(np.linspace(0,1,npoints),tck)
        
        self.setDestinations(x_i, y_i, reset)
        
        
    def makeBikeDrawing(self, ax, drawForce=False,
                                  drawPotential=False,
                                  drawTrajectory=False,
                                  drawNextDest=False,
                                  drawDestQueue=False,
                                  drawPastDest=False,
                                  drawName=False):
                                
        """Create a bicycle drawing on axes ax using current state 
        """
        
        self.hasDrawings[0] = True
        self.hasDrawings[1] = drawForce
        self.hasDrawings[2] = drawPotential 
        self.hasDrawings[3] = drawTrajectory
        self.hasDrawings[4] = drawNextDest                          
        self.hasDrawings[5] = drawDestQueue 
        self.hasDrawings[6] = drawPastDest
        self.hasDrawings[7] = drawName
        
        # calc keypoints
        x,y = self.bikeDrawingKeypoints()       
        
        # list to save graphic object handles
        self.ghandles = [None] * (15)
         
        #wheels             
        self.ghandles[0], = ax.plot(x[2:4], y[2:4], color='slategray', linewidth=4, animated=True)
        self.ghandles[1], = ax.plot(x[4:6], y[4:6], color="slategray", linewidth=4, animated=True)
        #frame
        self.ghandles[2], = ax.plot(x[0:2], y[0:2], 
                                  color=(230/255,134/255,55/255), linewidth=4, animated=True)
        self.ghandles[3], = ax.plot(x[6:8], y[6:8], 
                                  color=(230/255,134/255,55/255), linewidth=4, animated=True)
        #arms
        self.ghandles[4], = ax.plot([x[10], x[6]], [y[10], y[6]], 
                                  color=(0, 166/255, 214/255), linewidth=4, animated=True)
        self.ghandles[5], = ax.plot([x[11], x[7]], [y[11], y[7]], 
                                  color=(0, 166/255, 214/255), linewidth=4, animated=True)
        #body and head
        self.ghandles[6] = Ellipse((x[8],y[8]), 0.6, 0.4, toDeg(self.s[2]), 
                                   facecolor=(0, 166/255, 214/255), alpha=1, animated=True)
        self.ghandles[7] = Ellipse((x[9],y[9]), 0.3, 0.3, toDeg(self.s[2]), 
                                   facecolor=(0, 166/255, 214/255), alpha=1, animated=True)
        
        for i in range(0,7):
            ax.draw_artist(self.ghandles[i])
        #ax.draw_artist(self.ghandles[6])
        #ax.draw_artist(self.ghandles[7])
        
        #force
        if drawForce:
            #try:
            self.ghandles[8] = ax.arrow(self.s[0], 
                                        self.s[1], 
                                        self.force[0]/2, 
                                        self.force[1]/2,
                                        linewidth = 3,
                                        head_width=.4,
                                        head_length=.5,
                                        color=(230/255,134/255,55/255),
                                        animated=True)
            ax.draw_artist(self.ghandles[8])
            #except:
            #    xtxt = x[8] - 0.7 
            #    ytxt = y[8] - 1.5 
            #    self.ghandles[8] = ax.text(xtxt, ytxt, "arrived", fontsize=6)
                
        #potential   
        if drawPotential:
            a = .3 * self.PDECAY * np.log(2)
            b = .3 * a/np.sqrt(1-self.e**2)
            xpot = x[1] - self.e * np.cos(self.s[2]+self.s[4])
            ypot = y[1] - self.e * np.sin(self.s[2]+self.s[4])
            self.ghandles[9] = Ellipse((xpot,ypot), 2*a, 2*b, 
                                       toDeg(self.s[2]+self.s[4]), 
                                       facecolor="none", 
                                       edgecolor="slategray", 
                                       alpha=1,
                                       animated=True)
            ax.draw_artist(self.ghandles[9])
            #ax.draw_artist(self.ghandles[9])
         
        #trajectory
        if drawTrajectory:           
            self.ghandles[10], = ax.plot(self.traj[0], self.traj[1], 
                                             color="slategray", linewidth=3, animated=True)
            ax.draw_artist(self.ghandles[10])
            
        #next destination       
        if drawNextDest: 
            self.ghandles[11], = ax.plot((x[1], self.dest[0]), 
                                        (y[1], self.dest[1]), 
                                        color="slategray", 
                                        linewidth=3, 
                                        linestyle="dashed",
                                        animated=True)
            ax.draw_artist(self.ghandles[11])
            if not drawDestQueue:
                self.ghandles[12], \
                    = ax.plot(self.dest[0], self.dest[1], marker="x", 
                              markersize=10, 
                              markeredgecolor=(230/255,134/255,55/255),
                              markeredgewidth=2,
                              animated=True)
                ax.draw_artist(self.ghandles[12])
        #destination queue
        if drawDestQueue:
            #next and past destinations
            if self.destqueue is None: 
                self.ghandles[12], \
                    = ax.plot(self.dest[0], self.dest[1], marker="x", 
                              markersize=10, 
                              markeredgecolor=(230/255,134/255,55/255), 
                              markeredgewidth=2,
                              animated=True)
                ax.draw_artist(self.ghandles[12])
            else:
                self.ghandles[12], \
                    = ax.plot(self.destqueue[self.destpointer:,0], 
                              self.destqueue[self.destpointer:,1],
                              linestyle='None',
                              marker="x", 
                              markersize=10, 
                              markeredgecolor=(230/255,134/255,55/255),
                              markeredgewidth=2,
                              animated=True)
                ax.draw_artist(self.ghandles[12])
                    
        if drawPastDest and self.destqueue is not None:           
            self.ghandles[13], \
                = ax.plot(self.destqueue[:self.destpointer,0], 
                          self.destqueue[:self.destpointer,1], 
                          linestyle='None',
                          marker="x", 
                          markersize=10, 
                          markeredgecolor=(165/255,202/255,26/255),
                          markeredgewidth=2,
                          animated=True)
            ax.draw_artist(self.ghandles[13])
                
                
        if drawName:
            self.ghandles[14] = ax.text(self.s[0], self.s[1]+1, self.id, 
                                        color=(0, 166/255, 214/255), fontsize=20, animated=True)
            ax.draw_artist(self.ghandles[14])
        

    def bikeDrawingKeypoints(self):
        """Calc keypoints for a bike drawing
        """
        #frame
        xfm0 = self.s[0] 
        yfm0 = self.s[1] 
        xfm1 = self.s[0] + 1 * np.cos(self.s[2])
        yfm1 = self.s[1] + 1 * np.sin(self.s[2])
        
        # rear wheel
        xrw0 = xfm0 - 0.35 * np.cos(self.s[2])
        yrw0 = yfm0 - 0.35 * np.sin(self.s[2])
        xrw1 = xfm0 + 0.35 * np.cos(self.s[2])
        yrw1 = yfm0 + 0.35 * np.sin(self.s[2])
        
        
        # front wheel
        xfw0 = xfm1 - 0.35 * np.cos(self.s[2]+self.s[4])
        yfw0 = yfm1 - 0.35 * np.sin(self.s[2]+self.s[4])
        xfw1 = xfm1 + 0.35 * np.cos(self.s[2]+self.s[4])
        yfw1 = yfm1 + 0.35 * np.sin(self.s[2]+self.s[4])
        
        # handlebar
        xhb0 = xfm1 - 0.25 * np.cos(0.5*np.pi + self.s[2]+self.s[4])
        yhb0 = yfm1 - 0.25 * np.sin(0.5*np.pi + self.s[2]+self.s[4])
        xhb1 = xfm1 + 0.25 * np.cos(0.5*np.pi + self.s[2]+self.s[4])
        yhb1 = yfm1 + 0.25 * np.sin(0.5*np.pi + self.s[2]+self.s[4])
        
        # rider 
        xrb0 = xfm0 + 0.4 * np.cos(self.s[2])
        yrb0 = yfm0 + 0.4 * np.sin(self.s[2])
        xrh0 = xfm0 + 0.7 * np.cos(self.s[2])
        yrh0 = yfm0 + 0.7 * np.sin(self.s[2])
    
        
        # left arm 
        xla0 = xrb0 - 0.2 * np.cos(0.5*np.pi + self.s[2]+self.s[4])
        yla0 = yrb0 - 0.2 * np.sin(0.5*np.pi + self.s[2]+self.s[4])
    
        # left arm 
        xra0 = xrb0 + 0.2 * np.cos(0.5*np.pi + self.s[2]+self.s[4])
        yra0 = yrb0 + 0.2 * np.sin(0.5*np.pi + self.s[2]+self.s[4]) 
        
        return (xfm0, xfm1, xrw0, xrw1, xfw0, xfw1, 
                xhb0, xhb1, xrb0, xrh0, xla0, xra0), \
               (yfm0, yfm1, yrw0, yrw1, yfw0, yfw1, 
                yhb0, yhb1, yrb0, yrh0, yla0, yra0)
      

    def updateBikeDrawing(self, ax):
        """Update bicycle drawing according to the current state vector s
        """
        assert self.hasDrawings[0], \
            "Call makeBikeDrawing() before updateBikeDrawing()!"

        x,y = self.bikeDrawingKeypoints()   
        
        #wheels             
        self.ghandles[0].set_data(x[2:4], y[2:4])
        self.ghandles[1].set_data(x[4:6], y[4:6])
        #frame
        self.ghandles[2].set_data(x[0:2], y[0:2])
        self.ghandles[3].set_data(x[6:8], y[6:8])
        #arms
        self.ghandles[4].set_data([x[10], x[6]], [y[10], y[6]])
        self.ghandles[5].set_data([x[11], x[7]], [y[11], y[7]])
        #body and head
        self.ghandles[6].set_center((x[8],y[8]))
        self.ghandles[6].set_angle(toDeg(self.s[2]))
        self.ghandles[7].set_center((x[9],y[9]))
        self.ghandles[7].set_angle(toDeg(self.s[2]))
        
        for i in range(0,7):
            ax.draw_artist(self.ghandles[i])
        
        #self.hasDrawings = [drawForce, drawPotential, 
        #                    drawTrajectory, drawNextDest, 
        #                    drawDestQueue, drawPastDest]
        
        #force
        if self.hasDrawings[1]:
            #try:
            self.ghandles[8].set_data(x=self.s[0], 
                                      y=self.s[1], 
                                      dx=self.force[0]/2, 
                                      dy=self.force[1]/2)
            ax.draw_artist(self.ghandles[8])
            #except:
            #    self.ghandles[8].set_position(x[8]-0.7, y[8]-1.5)
        
        #potential
        if self.hasDrawings[2]:
            a = .3 * self.PDECAY * np.log(2)
            b = .3 * a/np.sqrt(1-self.e**2)
            xpot = x[1] - self.e * np.cos(self.s[2]+self.s[4])
            ypot = y[1] - self.e * np.sin(self.s[2]+self.s[4])
            
            self.ghandles[9].set_center((xpot,ypot))
            self.ghandles[9].set_angle(toDeg(self.s[2]+self.s[4]))
            self.ghandles[9].set_angle(toDeg(self.s[2]+self.s[4]))
            self.ghandles[9].set_width(2*a)
            self.ghandles[9].set_height(2*b)
            ax.draw_artist(self.ghandles[9])
        
        #trajectory
        if self.hasDrawings[3]: 
            self.ghandles[10].set_data(self.traj[0], self.traj[1])
            ax.draw_artist(self.ghandles[10])
            
        #next destination
        if self.hasDrawings[4]:
            self.ghandles[11].set_data((x[1], self.dest[0]), 
                                       (y[1], self.dest[1]),)
            ax.draw_artist(self.ghandles[11])
            
            if not self.hasDrawings[4]:
                self.ghandles[12].set_data(self.dest[0], self.dest[1])
                ax.draw_artist(self.ghandles[12])
                
        #destination queue
        if self.hasDrawings[5]:
            if self.destqueue is None: 
                self.ghandles[12].set_data(self.dest[0], self.dest[1])
            else:
                self.ghandles[12].set_data(self.destqueue[self.destpointer:,0], 
                                           self.destqueue[self.destpointer:,1])
            ax.draw_artist(self.ghandles[12])
            
        #past destinations
        if self.hasDrawings[6]:
            if self.ghandles[13] is not None:
                self.ghandles[13].set_data(self.destqueue[:self.destpointer,0], 
                                           self.destqueue[:self.destpointer,1])
            ax.draw_artist(self.ghandles[13])
            
        #name
        if self.hasDrawings[7]:
            self.ghandles[14].set_position((self.s[0], self.s[1]+1))
            ax.draw_artist(self.ghandles[14])  
            
    def endAnimation(self):
        """ End animation of the vehicle  
    
        Set the "animated" property of all graphic object to False to prevent
        them from disappearing once the animation ends. 

        Returns
        -------
        None.

        """    
        for g in self.ghandles:
            if g is not None:
                g.set_animated(False)
            