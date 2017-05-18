# -*- coding: utf-8 -*-
"""
Created on Sat May 13 21:16:09 2017

@author: ozer
"""
import numpy as np
import graphics as g

#POLY = [[59.360307, 18.052161],
#        [59.358581, 18.052697],
#        [59.359098, 18.054961],
#        [59.360558, 18.054811]]
#
#xmin,ymin = list(np.min(np.array(POLY),axis=0))
#xmax,ymax = list(np.max(np.array(POLY),axis=0))
#
##win = g.GraphWin('test',500,500)
##win.setCoords(minx,miny,maxx,maxy)
##
##while True:
##    mouse = win.checkMouse()
##    if mouse is not None:
##        print mouse
#
#from geopy.distance import vincenty
##print vincenty(POLY[0],POLY[1]).meters
#
#xmid = (xmin+xmax) /2.
#ymid = (ymin+ymax) /2.
#
#vincenty([xmax,ymid],[xmin,ymid]).meters
#vincenty([xmid,ymax],[xmid,ymin]).meters

import geometry as gm
import matplotlib.pyplot as plt

self_pos = np.array([0,0])
self_V = [1,0]
other_pos = np.array([1,1])
other_V = np.array([0,-1])
#vector that points at the other agent center from this center
center_vector = other_pos - self_pos
#distance between centers
dist = gm.euclid_distance(self_pos, other_pos)
#angle between center vector and the tangent line to other agent
theta = np.arcsin((1.)/dist)
#angle of the center vector
alpha = np.arcsin(center_vector[1]/dist)

#angle of the tangent lines
t1_a = alpha+theta
t2_a = alpha-theta

#make a really large triangle out of these two tangent lines to make
#checking inclusion easier.

#one side
t1 = np.array([np.cos(t1_a), np.sin(t1_a)])
t1 += self_pos
#other side
t2 = np.array([np.cos(t2_a), np.sin(t2_a)])
t2 += self_pos
#three vertices
VO = np.array([self_pos, t1, t2])
VO += other_V

plt.scatter([self_pos[0]],[self_pos[1]],c='red')
plt.scatter([other_pos[0]],[other_pos[1]],c='blue')
plt.plot(VO[:,0],VO[:,1])
plt.scatter(VO[:,0],VO[:,1],c='green',marker=',')
