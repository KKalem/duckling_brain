# -*- coding: utf-8 -*-
"""
Created on Tue Apr 11 19:10:28 2017

@author: ozer
"""

import numpy as np
import matplotlib.pyplot as plt

r = 1
pts = []
#for angle in np.linspace(0,np.pi, 180):
#    px,py = r*np.sin(angle), r*np.cos(angle)
#    pts.append((px,py))

pts = [[1,0],[1,1],[0,1],[-1,1],[-1,0],[-1,-1],[0,-1],[1,-1]]

pts = np.array(pts)

#plt.scatter(pts[:,0],pts[:,1])

import geometry as gm

heading = [1,0]

angles = []
for pt in pts:
    angles.append(gm.directed_angle(heading,pt))

plt.scatter(range(len(angles)),angles)