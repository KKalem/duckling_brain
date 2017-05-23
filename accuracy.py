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
import gp


trace0 = '_0_trace_'+config.SUFFIX
trace1 = '_1_trace_'+config.SUFFIX
trace = config.SUFFIX

m0 = np.loadtxt('traces/'+trace0)
m1 = np.loadtxt('traces/'+trace1)
to = 999999999
m = np.vstack((m0[:to],m1[:to]))


#%%
skip = 35
gpr = gp.GP()
gpr.fit(m[::skip])

fig, means, stds = gpr.show_surface(m, show=True, grid_density=200)
gpr.save_matrix(suffix=trace)
plt.matshow(stds)
plt.colorbar()

explored = stds<0.04
plt.matshow(explored.reshape(stds.shape))

#import sys
#sys.exit(0)
#%%
# load the regressed map and the true map
truth = np.load('traces/depthmap.npy')
means = np.load('traces/means'+trace+'.npy')
stds = np.load('traces/stds'+trace+'.npy')

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
#m[:,:2] *= config.PPM
#m[:,:2] += truth.shape[0]/2


mats = [truth, predictions, stds, sqe, abe]
cmaps = [colormap, colormap, 'Greens', 'Reds', 'Reds']
titles = ['Ground truth seabed',
          'Predicted seabed',
          'Prediction std dev.',
          'Squared error (mean='+str(mse)+')',
          'Absolute error (mean='+u.float_format2(mae)+')']
filenames = ['ground','predicted','std','sqe','abe']

#for i in range(len(mats)):
#for i in range(1,5):
for i in [1,2]:
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
    plt.plot(m[len(m0):,0],m[len(m0):,1], '--', color='magenta')
    plt.title(titles[i])
    plt.hold(False)
    plt.show()
    plt.savefig('figures/fig_'+filenames[i]+'.pdf')

#import geometry as gm
#L = 0
#ds0 = []
#ds1 = []
#for i in range(2,len(m0)):
#    d = gm.euclid_distance(m0[i-1,:2], m0[i,:2])
#    L += d
#    ds0.append(d)
#print(L)
#for i in range(2,len(m1)):
#    d = gm.euclid_distance(m1[i-1,:2], m1[i,:2])
#    L += d
#    ds1.append(d)
#print(L)
#print(ds0)
#print(ds1)
#print(sum(ds0))
#print(sum(ds1))
#
#print('Total travel:'+str(L))
#print('MSE:'+str(mse))
#print('MAE:'+str(mae))


#%%
#plt.figure()
#plt.xlim(0,truth.shape[0])
#plt.ylim(0,truth.shape[0])
#plt.imshow(truth, cmap=colormap)
#plt.colorbar()
#plt.title('Ground truth seabed')
#plt.savefig('ground.pdf')


#import geometry as gm
#
#traces = ['1A_sweeping/_0_trace___sweeping__',
#          '1A_tahir/_0_trace___1a_pure_tahirovic__',
#          '2A_tahir/_0_trace___2a_tahir__',
#          '2A_tahir/_1_trace___2a_tahir__',
#          '1A_greedy_009/_0_trace___1a_greedy9__',
#          '1A_greedy_013/_0_trace___1a_greedy13__',
#          '1A_gp_tahir_009/_0_trace___1a_gp_tahir__',
#          '1A_gp_tahir_013/_0_trace___1a_gp_tahir__',
#          '2A_greedy_009/_0_trace___2a_greedy__',
#          '2A_greedy_009/_1_trace___2a_greedy__',
#          '2A_greedy_013/_0_trace___2a_greedy__',
#          '2A_greedy_013/_1_trace___2a_greedy__',
#          '2A_gp_tahir_009/_0_trace___2a_gp_tahir_009__',
#          '2A_gp_tahir_009/_1_trace___2a_gp_tahir_009__',
#          '2A_gp_tahir_013/_0_trace___2a_gp_tahir_013__',
#          '2A_gp_tahir_013/_1_trace___2a_gp_tahir_013__',
#          'com_2A_gp_tahir_009/_0_trace___com_gp_tahir_009__',
#          'com_2A_gp_tahir_009/_1_trace___com_gp_tahir_009__',
#          'com_2A_greedy_009/_0_trace___com_greedy_009__',
#          'com_2A_greedy_009/_1_trace___com_greedy_009__',
#          'com_2A_tahir/_0_trace___com_tahir__',
#          'com_2A_tahir/_1_trace___com_tahir__'
#          ]
#
#for trace in traces:
#    print(trace)
#    m = np.loadtxt('figures/'+trace)
#    L = 0
#    for i in range(1,len(m)):
#        d = gm.euclid_distance(m[i-1,:2], m[i,:2])
#        L += d
#    print(L)
