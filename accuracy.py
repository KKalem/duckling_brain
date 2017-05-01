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

suffix = '_May1'
# load the regressed map and the true map
truth = np.load(config.TRACE_DIR+'/depthmap.npy')
means = np.load(config.TRACE_DIR+'/means'+suffix+'.npy')
stds = np.load(config.TRACE_DIR+'/stds'+suffix+'.npy')

# load the measurement trace for min/max values
m = u.load_trace('1')
m = np.array(m)

#s = 10*config.PPM
#
##slice the matrices into the area that the boat was allowed to operate in
#truth = truth[s:-s,s:-s]
#means = means[s:-s,s:-s]
#stds = stds[s:-s,s:-s]


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
titles = ['Ground truth seabed', 'Predicted seabed', 'Prediction variance','Squared error (mean='+str(mse)+')', 'Absolute error (mean='+u.float_format2(mae)+', median='+u.float_format2(medae)+')']

#for i in range(len(mats)):
for i in range(5):
    plt.figure()
    plt.xlim(0,truth.shape[0])
    plt.ylim(0,truth.shape[0])
    plt.hold(True)
    plt.imshow(mats[i], cmap = cmaps[i])
    plt.colorbar()
    #FOR SOME REASON x-y is switched. /shrug
    plt.plot(m[:,1],m[:,0], color='magenta')
    plt.title(titles[i])
    plt.hold(False)
    plt.show()
    plt.savefig('etc/fig_'+titles[i]+'.png')

import geometry as gm
L = 0
for i in range(2,len(m)):
    L += gm.euclid_distance(m[i-1,:2], m[i,:2])
print('Total travel:'+str(L))
