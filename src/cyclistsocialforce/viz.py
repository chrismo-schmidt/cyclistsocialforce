# -*- coding: utf-8 -*-
"""
Created on Mon Apr 24 13:21:14 2023

Will replace the graphic functions in vehicle.py. TODO!

@author: Christoph Schmidt
"""
import numpy as np
from matplotlib.patches import Ellipse
from cyclistsocialforce.utils import toDeg

class BikeDrawing: 

    def makeBikeDrawing(self, ax, drawForce=False,
                                  drawPotential=False,
                                  drawTrajectory=False,
                                  drawNextDest=False,
                                  drawDestQueue=False,
                                  drawPastDest=False):
                                
        """Create a bicycle drawing on axes ax using current state 
        """
        
        self.hasDrawings = [drawForce, drawPotential, 
                            drawTrajectory, drawNextDest, 
                            drawDestQueue, drawPastDest]
        
        # calc keypoints
        x,y = self.bikeKeypoints()       
        
        # list to save graphic object handles
        self.ghandles = [None] * (22)
         
        #wheels             
        self.ghandles[0] = ax.plot(x[2:4], y[2:4], color='black', linewidth=2)
        self.ghandles[1] = ax.plot(x[4:6], y[4:6], color='black', linewidth=2)
        #frame
        self.ghandles[2] = ax.plot(x[0:2], y[0:2], 
                                  color=(230/255,134/255,55/255), linewidth=1)
        self.ghandles[3] = ax.plot(x[6:8], y[6:8], 
                                  color=(230/255,134/255,55/255), linewidth=1)
        #arms
        self.ghandles[4] = ax.plot([x[10], x[6]], [y[10], y[6]], 
                                  color=(0, 166/255, 214/255), linewidth=2)
        self.ghandles[5] = ax.plot([x[11], x[7]], [y[11], y[7]], 
                                  color=(0, 166/255, 214/255), linewidth=2)
        #body and head
        self.ghandles[6] = Ellipse((x[8],y[8]), 0.6, 0.4, toDeg(self.s[2]), 
                                   facecolor=(0, 166/255, 214/255), alpha=1)
        self.ghandles[7] = Ellipse((x[9],y[9]), 0.3, 0.3, toDeg(self.s[2]), 
                                   facecolor="black", alpha=1)
        
        ax.add_artist(self.drawing[6])
        ax.add_artist(self.drawing[7])
        
        #force
        if drawForce:
            try:
                self.ghandles[8] = ax.arrow(self.s[0], self.s[1], 
                                             self.force[0], self.force[1])
            except:
                xtxt = x[8] - 0.7 
                ytxt = y[8] - 1.5 
                self.ghandles[8] = ax.text(xtxt, ytxt, "arrived", fontsize=6)
                
        #potential   
        if drawPotential:
            b = self.PDECAY * np.log(2)
            a = b/np.sqrt(1-self.e**2)
            xpot = x[1] - self.e * np.cos(self.s[2]+self.s[4])
            ypot = y[1] - self.e * np.sin(self.s[2]+self.s[4])
            potential = Ellipse((xpot,ypot), 2*a, 2*b, 
                                toDeg(self.s[2]+self.s[4]), 
                                facecolor="none", edgecolor="slategray", 
                                alpha=1)
            self.ghandles[9] = potential
            ax.add_artist(self.potentialDrawing)
         
        #trajectory
        if drawTrajectory:           
            self.ghandles[10] = ax.plot(self.traj[0], self.traj[1], 
                                             color="slategray", linewidth=.5)
        #next destination       
        if drawNextDest: 
            self.ghandles[11] = ax.plot((x[1], self.dest[0]), 
                                        (y[1], self.dest[1]), 
                                        color="slategray", 
                                        linewidth=.5, 
                                        linestyle="dashed")
            if not drawDestQueue:
                self.ghandles[12] \
                    = ax.plot(self.dest[0], self.dest[1], marker="x", 
                              markersize=5, 
                              markeredgecolor=(230/255,134/255,55/255))
        #destination queue
        if drawDestQueue:
            #next and past destinations
            if self.destqueue is None: 
                self.ghandles[12] \
                    = ax.plot(self.dest[0], self.dest[1], marker="x", 
                              markersize=5, 
                              markeredgecolor=(230/255,134/255,55/255))
            else:
                self.ghandles[12] \
                    = ax.plot(self.destqueue[self.destpointer:,0], 
                              self.destqueue[self.destpointer:,1],
                              linestyle='None',
                              marker="x", 
                              markersize=5, 
                              markeredgecolor=(230/255,134/255,55/255))
                    
        if drawPastDest and self.destqueue is not None:           
            self.ghandles[13] \
                = ax.plot(self.destqueue[:self.destpointer,0], 
                          self.destqueue[:self.destpointer,1], 
                          linestyle='None',
                          marker="x", 
                          markersize=5, 
                          markeredgecolor=(165/255,202/255,26/255))
        

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
      

    def updateBikeDrawing(self):
        """Update bicycle drawing a new state vector s
        """
        assert self.hasDrawings[0], \
            "Call makeBikeDrawing() before updateBikeDrawing()!"

        x,y = self.bikeKeypoints()   
        
        #wheels             
        self.ghandles[0].set_data(x[2:4], y[2:4])
        self.ghandles[1].set_data(x[4:6], y[4:6])
        #frame
        self.ghandles[2].set_data(x[0:2], y[0:2])
        self.ghandles[3].set_data(x[6:8], y[6:8])
        #arms
        self.ghandles[4].set_data([x[10], x[6]])
        self.ghandles[5].set_data([x[11], x[7]])
        #body and head
        self.ghandles[6].set_center((x[8],y[8]))
        self.ghandles[6].set_angle(toDeg(self.s[2]))
        self.ghandles[7].set_center((x[9],y[9]))
        self.ghandles[7].set_angle(toDeg(self.s[2]))
        
        
        #self.hasDrawings = [drawForce, drawPotential, 
        #                    drawTrajectory, drawNextDest, 
        #                    drawDestQueue, drawPastDest]
        

        #force
        if self.hasDrawings[0]:
            try:
                self.ghandles[8].set_data(self.s[0], self.s[1], 
                                             self.force[0], self.force[1])
            except:
                self.ghandles[8].set_position(x[8]-0.7, y[8]-1.5)
        
        #potential
        if self.hasDrawings[1]:
            b = self.PDECAY * np.log(2)
            a = b/np.sqrt(1-self.e**2)
            xpot = x[1] - self.e * np.cos(self.s[2]+self.s[4])
            ypot = y[1] - self.e * np.sin(self.s[2]+self.s[4])
            
            self.ghandles[9].set_center((xpot,ypot))
            self.ghandles[9].set_angle(toDeg(self.s[2]+self.s[4]))
            self.ghandles[9].set_angle(toDeg(self.s[2]+self.s[4]))
            self.ghandles[9].set_width(2*a)
            self.ghandles[9].set_hight(2*b)
        
        #trajectory
        if self.hasDrawingsl[2]: 
            self.ghandles[10].set_data(self.traj[0], self.traj[1])
        
        #next destination
        if self.hasDrawings[3]:
            self.ghandles[11].set_data((x[1], self.dest[0]), 
                                       (y[1], self.dest[1]),)
            if not self.hasDrawings[4]:
                self.ghandles[12].set_data(self.dest[0], self.dest[1])
        
        #destination queue
        if self.hasDrawings[4]:
            if self.destqueue is None: 
                self.ghandles[12].set_data(self.dest[0], self.dest[1])
            else:
                self.ghandles[12].set_data(self.destqueue[self.destpointer:,0], 
                                           self.destqueue[self.destpointer:,1])
        
        #past destinations
        if self.hasDrawings[5]:
            self.ghandles[13].set_data(self.destqueue[:self.destpointer,0], 
                                       self.destqueue[:self.destpointer,1])

        