# -*- coding: utf-8 -*-
"""
Created on Wed Apr 19 16:49:49 2023

@author: Christoph Schmidt
"""

import numpy as np

def toDeg(rad):
    return 360*rad/(2*np.pi)

def toRad(deg):
    return 2*np.pi*deg/(360)

def clearAxes(ax):
    
    for e in ax.get_children():
        e.remove()
        
def angleSUMOtoSFM(theta):
    """Convert angle between SUMO definition and SFM definition
    """
    return limitAngle((np.pi/2)-toRad(theta))

def angleSFMtoSUMO(theta):
    """Convert angle between SUMO definition and SFM definition
    """
    return toDeg(expandAngle((np.pi/2)-theta))
    
      
def limitAngle(theta):
    """Convert angle from [0,2*pi] to [-pi,pi]
    """
    if isinstance(theta, np.ndarray):
        theta = (np.floor(theta/(2*np.pi)) * (- 2 * np.pi) + theta)
        
        theta[theta > np.pi] = (theta - 2*np.pi)[theta > np.pi]
        theta[theta < -np.pi] = (theta + 2*np.pi)[theta < -np.pi]
    else:
        theta = (np.floor(theta/(2*np.pi)) * (- 2 * np.pi) + theta)
        
        if theta > np.pi:
            theta = theta - 2*np.pi
        elif theta < -np.pi:
            theta = theta + 2*np.pi
    
    return theta

def expandAngle(theta):
    """Convert angle from [-pi,pi] [0,2*pi] to
    """
    
    if theta < 0 :
        theta = 2*np.pi + theta
        
    return theta

def angleDifference(a1, a2):
    
    if isinstance(a1, np.ndarray):
        da=np.zeros_like(a1)
        
        da[a1 > a2] = (a1-a2)[a1 > a2]
        da[a1 <= a2] = (a2-a1)[a1 <= a2]
     
        da[da>np.pi] = (2*np.pi) - da[da>np.pi]
    
        test_1 = np.abs(limitAngle(a1 - da) - a2)
        test_2 = np.abs(limitAngle(a1 + da) - a2)
    
        da[test_1 < test_2] = -da[test_1 < test_2]

        return da
    
    else:
        if a1 > a2:
            da = a1 - a2
        else:
            da = a2 - a1
        
        if da > np.pi:
            da = (2*np.pi) - da
            
        test_1 = abs(limitAngle(a1 - da) - a2)
        test_2 = abs(limitAngle(a1 + da) - a2)
        
        if test_1 < test_2:
            return -da 
        else:
            return da

def cart2polar(x,y):
    
    rho = np.sqrt(np.power(x,2)+np.power(y,2))
    
    #phi = np.arccos(x/rho)
    #phi[y<0] = (2*np.pi) - phi[y<0]
    
    phi = np.arccos(x/rho)
    phi[y<0] = - phi[y<0]
    
    return rho, phi

def polar2cart(rho,phi):

    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
        
    return x, y

def thresh(x, minmax):
    if x < minmax[0]:
        x = minmax[0]
    elif x > minmax[1]:
        x = minmax[1]
    return x