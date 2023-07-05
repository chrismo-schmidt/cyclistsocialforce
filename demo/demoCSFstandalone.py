# -*- coding: utf-8 -*-
"""Run a cyclist social force demo without SUMO.

Simulates two encroaching cyclists and plots an example of force and potential. 

usage: demoCSFstandalone.py [-h] [-s]

optional arguments:
  -h, --help  show this help message and exit
  -s, --save  Save results to ./output/

Created on Tue Feb 14 18:26:19 2023
@author: Christoph Schmidt
"""
import numpy as np
import matplotlib.pyplot as plt
import argparse

from datetime import datetime
from time import sleep,time
from os import mkdir, path


from cyclistsocialforce.vehicle import Vehicle
from cyclistsocialforce.intersection import SocialForceIntersection


def initAnimation():
    """ Initialize the animation of the demo. 
    
    Use blitting for speed-up. 

    Returns
    -------
    fig : figure handle
        The figure the animation will be shown in 
    fig_bg : image
        The background of the figure for blitting 
    ax : axes object
        The axes the animation will be shown in 

    """
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    figManager = plt.get_current_fig_manager()
    figManager.resize(500, 500)
    plt.show(block=False)
    plt.pause(.1)
    fig_bg = fig.canvas.copy_from_bbox(fig.bbox)
    fig.canvas.blit(fig.bbox)
    
    return fig, fig_bg, ax


def run(intersection, fig, fig_bg):
    """Run the simulation of the demo intersection
    """

    i = 0
    while i < 200:
        
        t = time()
        
        fig.canvas.restore_region(fig_bg)
        
        intersection.step()       
    
        fig.canvas.blit(fig.bbox)
        fig.canvas.flush_events()
        
        dt = (time()-t)
        if dt<0.01:
            sleep(0.01-dt)
            rate = 1.0
        else:
            rate = 0.01/dt  
            
        i = i + 1
    
    # End animation to prevent animated graphics from disappearing.
    intersection.endAnimation()
    
def plotPotentialsAndForcefields(bike1, bike2, dirname=None):
     lnspx = np.arange(0, 12, 0.5)
     lnspy = np.arange(0, 12, 0.5)
     X1, Y1 = np.meshgrid(lnspx, lnspy)
     
     lnspx = np.arange(0, 12, 0.1)
     lnspy = np.arange(0, 12, 0.1)
     X2, Y2 = np.meshgrid(lnspx, lnspy)
     
     Fx1, Fy1 = bike1.calcRepulsiveForce(X1, Y1)
     Fx2, Fy2 = bike2.calcRepulsiveForce(X1, Y1)
     P1 = bike1.calcPotential(X2, Y2)
     P2 = bike2.calcPotential(X2, Y2)
     
     # Forcefield bike 1
     fig = plt.figure()
     ax1 = fig.add_subplot(2, 2, 1)
     ax1.set_aspect("equal")
     ax1.set_xlabel(r'$\frac{\it{x}}{\mathrm{m}}$')
     ax1.set_ylabel(r'$\frac{\it{y}}{\mathrm{m}}$')    
     ax1.quiver(X1, Y1, Fx1, Fy1)
     
     # Forcefield bike 2
     ax2 = fig.add_subplot(2, 2, 2)
     ax2.set_aspect("equal")
     ax2.set_xlabel(r'$\frac{\it{x}}{\mathrm{m}}$')
     ax2.set_ylabel(r'$\frac{\it{y}}{\mathrm{m}}$')     
     ax2.quiver(X1, Y1, Fx2, Fy2)
     
     # Potential bike 1
     ax3 = fig.add_subplot(2, 2, 3)
     ax3.set_aspect("equal")
     ax3.set_xlabel(r'$\frac{\it{x}}{\mathrm{m}}$')
     ax3.set_ylabel(r'$\frac{\it{y}}{\mathrm{m}}$')     
     ax3.imshow(P1, origin='lower')
     
     # Potential bike 2
     ax4 = fig.add_subplot(2, 2, 4)
     ax4.set_aspect("equal")
     ax4.set_xlabel(r'$\frac{\it{x}}{\mathrm{m}}$')
     ax4.set_ylabel(r'$\frac{\it{y}}{\mathrm{m}}$')
     ax4.imshow(P2, origin='lower')
    
     plt.tight_layout()
    
     if dirname is not None:
         plt.savefig(path.join(dirname,'potentialsAndForces.pdf'),
                     format="pdf")
        
     plt.show()

def parseArgs():
    parser = argparse.ArgumentParser(description='Run a cyclist social force '\
                                     'demo without SUMO.')
    parser.add_argument('-s','--save', dest='save', default=False, 
                        action='store_true', help='Save results to ./output/')
    return parser.parse_args()

def main():
    """Run a cyclist social force demo without SUMO. 
    
    This script:
        - creates some bicycles
        - gives them a destination
        - simulates their movement to the destinations
        - optionally shows:
            - force fields
            - potentials
            - force magnitude time histories        
    """
    args = parseArgs()
    
    if args.save:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        dirname = path.join("output", timestamp + "_testCyclistSocialForce")
        mkdir(dirname)
    else:
        dirname = None
    
    plt.rcParams['text.usetex'] = False
    
        
    bike1 = Vehicle((1, 8, 0 , 2, 0), userId="b1")
    bike2 = Vehicle((9.9, 2, np.pi/2, 5, 0), userId="b2")

    # A single destination for bike 1
    bike1.setDestinations(11,8)

    # A series of destinations for bike 2
    bike2.setSplineDestinations((10,4,3.5),(5,10,10), 5)
    
    # Initialize the animation. If the simulation is run with SUMO, the 
    # Scenario class takes care of that. Whithout Scenario, we have to do 
    # it manually. TODO: Generalize Scenario class.  
    fig, fig_bg, ax = initAnimation()

    # A social force intersection to manage the two bicycles. Deactivate Traci
    # to run the simulation without SUMO. Activate animation for some nice 
    # graphics. 
    intersection = SocialForceIntersection((bike1, bike2), use_traci=False, 
                                           animate=True, axes=ax)
    
    # Run the simulation
    run(intersection, fig, fig_bg)
    
    # Generate other output graphics
    plotPotentialsAndForcefields(bike1, bike2, dirname)

   
# Entry point
if __name__ == "__main__":
    main()
    