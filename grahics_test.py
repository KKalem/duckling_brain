# -*- coding: utf-8 -*-
"""
Created on Fri Mar  3 15:39:46 2017

@author: ozer
"""
from __future__ import print_function
import graphics as g
import numpy as np

win = g.GraphWin('test',500,500, autoflush=False)
img = g.Image(g.Point(0,0),500,500)

img.draw(win)

for i in range(100):
    img.setPixel(250+i,250+i,'black')


