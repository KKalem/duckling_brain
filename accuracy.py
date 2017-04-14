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

# load the regressed map and the true map
truth = np.load(config.TRACE_DIR+'/depthmap.npy')
means = np.load(config.TRACE_DIR+'/means_mat.npy')
stds = np.load(config.TRACE_DIR+'/stds_mat.npy')

# load the measurement trace for min/max values
m = u.load_trace('0')
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

plt.matshow(truth, cmap = colormap)
plt.title('Ground truth seabed')
plt.colorbar()

plt.matshow(predictions, cmap = colormap)
plt.title('Predicted seabed')
plt.colorbar()

plt.matshow(stds, cmap = 'Greens')
plt.title('Prediction variance')
plt.colorbar()

plt.matshow(sqe, cmap = 'Reds')
plt.title('Squared error (MSE='+str(mse)+')')
plt.colorbar()

plt.matshow(abe, cmap = 'Reds')
plt.title('Absolute error (MAE='+str(mae)+')')
plt.colorbar()