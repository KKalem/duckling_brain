# -*- coding: utf-8 -*-
"""
Created on Fri Mar  3 15:39:46 2017

@author: ozer
"""
from __future__ import print_function
import graphics as g

try:
    win = g.GraphWin('test',500,500, autoflush=False)

    c = g.Circle(g.Point(0,0), 20)
    c.draw(win)
    i = 0
    while True:
        g.update(30)
        mouse = win.checkMouse()
        if mouse is not None:
            x,y = mouse.getX(), mouse.getY()
            c._reconfig(center, (x,y))

except:
    win.close()

win.close()
