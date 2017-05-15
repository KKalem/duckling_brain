# -*- coding: utf-8 -*-
"""
Created on Fri Apr 14 13:14:10 2017

@author: ozer

Simple script to display and compare depthmap matrices
"""
from __future__ import print_function

import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

import config
import util as u

suffix = '_May15_FIXED'+config.SUFFIX
# load the regressed map and the true map
truth = np.load(config.TRACE_DIR+'/depthmap.npy')
means = np.load(config.TRACE_DIR+'/means_May15_FIXEDphysical.npy')
stds = np.load(config.TRACE_DIR+'/stds_May15_FIXEDphysical.npy')

# load the measurement trace for min/max values
m0 = np.loadtxt('traces/_0_trace_physical')
m1 = np.loadtxt('traces/_1_trace_physical')
m = np.vstack([m0,m1])


#scale the normalized means
predictions = u.scale_range(means, np.min(m[:,2]), np.max(m[:,2]))

#squared error
sqe = np.square(truth - predictions)
mse = np.mean(sqe)

#abolsute error
abe = np.abs(truth - predictions)
mae = np.mean(abe)
medae = np.median(abe)

colormap = mpl.colors.ListedColormap(u.load_colormap(config.COLORMAP_FILE))

#scale and shift the trace to fit plots
m[:,:2] *= config.PPM
m[:,:2] += truth.shape[0]/2


mats = [truth, predictions, stds, sqe, abe]
cmaps = [colormap, colormap, 'Greens', 'Reds', 'Reds']
titles = ['Ground truth seabed', 'Predicted seabed', 'Prediction std dev.','Squared error (mean='+str(mse)+')', 'Absolute error (mean='+u.float_format2(mae)+', median='+u.float_format2(medae)+')']

#for i in range(len(mats)):
for i in range(5):
    plt.figure()
    plt.xlim(0,truth.shape[0])
    plt.ylim(0,truth.shape[0])
    plt.hold(True)
    if i==2:
        plt.imshow(mats[i].T, cmap = cmaps[i])
    else:
        plt.imshow(mats[i], cmap = cmaps[i])
    plt.colorbar()
    plt.plot(m[:len(m0),0],m[:len(m0),1], color='magenta')
    plt.plot(m[len(m0):,0],m[len(m0):,1], color='magenta')
    plt.title(titles[i])
    plt.hold(False)
    plt.show()
    plt.savefig('etc/fig_'+titles[i]+'.png')

import geometry as gm
L = 0
for i in range(2,len(m)):
    L += gm.euclid_distance(m[i-1,:2], m[i,:2])
print('Total travel:'+str(L))
