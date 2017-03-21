# -*- coding: utf-8 -*-
"""
Created on Fri Mar  3 15:39:46 2017

@author: ozer
"""

import graphics as g

try:
    win = g.GraphWin('test',500,500, autoflush=False)

    c = g.Circle(g.Point(0,0), 20)
    c.draw(win)
    i = 0
    while True:
#        c.move(1,1)
        i+=1
        print i
        g.update(10)

except:
    win.close()

win.close()
