# -*- coding: utf-8 -*-
"""
Created on Thu Apr 20 10:19:21 2023

@author: Christoph Schmidt
"""

import sys
import numpy as np
import sumolib
import traci
import traceback

from math import sqrt, floor, ceil

from cyclistsocialforce.vehicle import Vehicle
from cyclistsocialforce.intersection import SocialForceIntersection
from cyclistsocialforce.utils import angleSUMOtoSFM

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from time import time, sleep

class Scenario:
    
    def __init__(self, network_file, animate=False):
        """Create a Scenario object based on a SUMO network file (.net.xml)

        Parameters
        ----------
        network_file : str
            Path + filename to a SUMO network file
        """
        
        #import network file 
        self.intersections = []
        net = sumolib.net.readNet(network_file)
        nodes = net.getNodes()
        
        #count nodes that are not dead ends
        n=0
        for i in range(len(nodes)):
            if len(nodes[i].getIncoming()) < 2 and \
               len(nodes[i].getOutgoing()) < 2:
                continue
            n += +1
        
        #set up animation
        self.animate = animate
        if self.animate:
            rows = int(np.floor(n/2))+1
            cols = int(np.ceil(n/2))+1
            self.fig = plt.figure()
        j = 1
        
        ninter = len(nodes)
        nrows = floor(sqrt(ninter))
        ncols = ceil(sqrt(ninter))
        
        #create intersections for SFM modelling
        for i in range(ninter):
            #only include nodes that are not dead ends
            if len(nodes[i].getIncoming()) < 2 and \
               len(nodes[i].getOutgoing()) < 2:
                continue
            
            if self.animate:
                ax = self.fig.add_subplot(nrows, ncols, j)
                j += 1
                
                self.intersections.append(SocialForceIntersection([], 
                                                            animate=True,
                                                            axes=ax,
                                                            use_traci=True,                                                           insId=nodes[i].getID(),
                                                            node=nodes[i]))
                figManager = plt.get_current_fig_manager()
                figManager.resize(960, 1080)
                plt.show(block=False)
                plt.pause(.1)
                self.fig_bg = self.fig.canvas.copy_from_bbox(self.fig.bbox)
                self.fig.canvas.blit(self.fig.bbox)
            else:
                self.intersections.append(SocialForceIntersection([],
                                                            use_traci=True,
                                                            insId=nodes[i].getID(),
                                                            node=nodes[i]))
            
        
            
    def addIntersection(self, i):
        self.intersections.append(i)
        
    def allocateRUs(self):     
        
        # road users that are already managed by an intersection
        alreadyOnIntersection = []
        for j in range(len(self.intersections)):  
            self.intersections[j].removeExited()
            alreadyOnIntersection += self.intersections[j].getRoadUserIDs()
            
        # get road users that are not managed by an intersection and their pos.
        ruids = list(traci.vehicle.getIDList())        
        ruids = np.setdiff1d(ruids, alreadyOnIntersection).tolist()
        rulocs = np.zeros((len(ruids),2))
        for i in range(len(ruids)):
            x,y = traci.vehicle.getPosition(ruids[i])
            rulocs[i,0] = x
            rulocs[i,1] = y
            
        # find road users on intersections
        onIntersection = np.full((len(ruids), len(self.intersections)), False)  

        for j in range(len(self.intersections)):    
            
            onIntersection[:,j] = \
                self.intersections[j].areOnIntersection(rulocs)
                
        # add road that have just entered to intersections          
        for i in range(len(ruids)): 
            if np.any(onIntersection[i,:]):
                j = np.argwhere(onIntersection[i,:])
                assert len(j)==1, "Road user on more then one intersection!"
                j=j[0,0]
                
                if not self.intersections[j].hasRoadUser(ruids[i]):                   
                    route = traci.vehicle.getRoute(ruids[i])
                    route = route[traci.vehicle.getRouteIndex(ruids[i]):]
                    
                    unew = Vehicle((rulocs[i,0],
                                    rulocs[i,1],
                                    angleSUMOtoSFM(traci.vehicle.getAngle(ruids[i])),
                                    traci.vehicle.getSpeed(ruids[i]),
                                    0),
                                    ruids[i],
                                    route)
                    self.intersections[j].addRoadUser(unew)

    def step(self, i):
        """Simulate a C-SFM step for all intersections in the scenario
        """
        
        #Allocate road users on intersection to simulation by the C-SFM model
        #and road users on links to simulation by SUMO
        self.allocateRUs()
        
        #Simulate intersections
        if self.animate: 
            t = time()
            self.fig.canvas.restore_region(self.fig_bg)
        for ins in self.intersections:       
            ins.step()
        if self.animate:
            self.fig.canvas.blit(self.fig.bbox)
            self.fig.canvas.flush_events()
            dt = (time()-t)
            if dt<0.01:
                sleep(0.01-dt)
                rate = 1.0
            else:
                rate = 0.01/dt  
            #print(f"{rate:.2f}")
            #print(f"{0.1/dt:.2f}", end="\r")

        #SUMO step
        traci.simulationStep()
        
        if not self.animate:
            return i+1
        
    def run(self):
        """Run scenario simulation
        """
        
        try:
            i = 0
            while traci.simulation.getMinExpectedNumber() > 0:
                self.step(i)
                i = i+1
        except Exception:
            print(traceback.format_exc())
        finally:   
            traci.close()
            sys.stdout.flush()

        
def step(i, scenario):
    """Simulate a C-SFM step for all intersections in the scenario
    """
    
    #Allocate road users on intersection to simulation by the C-SFM model
    #and road users on links to simulation by SUMO
    scenario.allocateRUs()
    
    #Simulate intersections
    for ins in scenario.intersections:
        ins.step()
        
    #SUMO step
    traci.simulationStep()
    
    if not scenario.animate:
        return i+1
        
                
        