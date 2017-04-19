# -*- coding: utf-8 -*-
"""
Created on Tue Apr 11 19:10:28 2017

@author: ozer
"""

import numpy as np
import matplotlib.pyplot as plt

#r = 1
#pts = []
##for angle in np.linspace(0,np.pi, 180):
##    px,py = r*np.sin(angle), r*np.cos(angle)
##    pts.append((px,py))
#
#pts = [[1,0],[1,1],[0,1],[-1,1],[-1,0],[-1,-1],[0,-1],[1,-1]]
#
#pts = np.array(pts)
#
##plt.scatter(pts[:,0],pts[:,1])
#
#import geometry as gm
#
#heading = [1,0]
#
#angles = []
#for pt in pts:
#    angles.append(gm.directed_angle(heading,pt))
#
#plt.scatter(range(len(angles)),angles)

#headings = np.linspace(0,2*np.pi,360)
#
#vx,vy = [],[]
#
#for h in headings:
#    vx.append(np.cos(h))
#    vy.append(np.sin(h))
#
#plt.plot(vx,'r')
#plt.plot(vy)


#def gem(l):
#    enough = 165.
#    take_last = 20
#    if len(l) > enough:
#        print 'larger'
#        #always take the last 20 with some skip
#        skip_last = 3
#        take_last *= skip_last
#        res = l[-take_last::skip_last]
#        print len(res)
#        skip = np.round(  (len(l)-len(res)) / enough   )
#        print skip
#        skip = max(1,skip)
#        res.extend(l[::int(skip)])
#        return res
#    else:
#        print 'smaller'
#        return l


import Agent

a = Agent.Agent('0', win=False, procs=False)

messes = []
for m_count in range(1000):
#    m_count = 200
    ms = zip(range(m_count),range(m_count),range(m_count))
    oms = zip(range(m_count),range(m_count),range(m_count),range(m_count),range(m_count))

    a.measurements = ms

    other_agents = {}
    for i in range(4):
        agent = {'id':i, 'mments':{}}
        for m in oms:
            agent['mments'][m[0]]=m
        other_agents[str(i)] = agent

    a.other_agents = other_agents

    mes = a.get_enough_measurements(enough=250)
    messes.append(len(mes))

plt.plot(messes)
#print len(mes)