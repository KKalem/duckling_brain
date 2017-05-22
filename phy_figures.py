# -*- coding: utf-8 -*-
"""
Created on Mon May 22 18:24:36 2017

@author: ozer
"""
from __future__ import print_function

import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

import config
import util as u
import gp


traces = ['phy_4','phy_5','phy_6']

m = np.loadtxt('traces/physical/'+'phy_3')
for trace in traces:
    m = np.vstack((m,np.loadtxt('traces/physical/'+trace)))

gpr = gp.GP()
gpr.fit(m[::3])

#%%
fig, means, stds = gpr.show_surface(m, show=True, grid_density=60, separate=True)