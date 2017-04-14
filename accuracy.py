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

suffix = '_14Apr'
# load the regressed map and the true map
truth = np.load(config.TRACE_DIR+'/depthmap.npy')
means = np.load(config.TRACE_DIR+'/means'+suffix+'.npy')
stds = np.load(config.TRACE_DIR+'/stds'+suffix+'.npy')

# load the measurement trace for min/max values
m = u.load_trace('1')
m = np.array(m)


#scale the normalized means
o_min = np.min(means)
o_max = np.max(means)
n_min = np.min(m[:,2])
n_max = np.max(m[:,2])
o_range = o_max - o_min
n_range = n_max - n_min

predictions = (((means - o_min) * n_range) / o_range) + n_min

#squared error
sqe = np.square(truth - predictions)
mse = np.mean(sqe)

#abolsute error
abe = np.abs(truth - predictions)
mae = np.mean(abe)

colormap = mpl.colors.ListedColormap(u.load_colormap(config.COLORMAP_FILE))

#scale and shift the trace to fit plots
m[:,:2] *= config.PPM
m[:,:2] += truth.shape[0]/2


mats = [truth, predictions, stds, sqe, abe]
cmaps = [colormap, colormap, 'Greens', 'Reds', 'Reds']
titles = ['Ground truth seabed', 'Predicted seabed', 'Prediction variance','Squared error (MSE='+str(mse)+')', 'Absolute error (MAE='+str(mae)+')']

for i in range(len(mats)):
    plt.figure()
    plt.xlim(0,truth.shape[0])
    plt.ylim(0,truth.shape[0])
    plt.hold(True)
    plt.imshow(mats[i], cmap = cmaps[i])
    plt.colorbar()
    plt.plot(m[:,0],m[:,1], color='magenta')
    plt.title(titles[i])
    plt.hold(False)
    plt.show()