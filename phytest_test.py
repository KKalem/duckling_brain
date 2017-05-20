# -*- coding: utf-8 -*-
"""
Created on Fri May 19 15:25:10 2017

@author: ozer
"""

import numpy as np
import matplotlib.pyplot as plt

m1 = np.load('traces/means_PHYTEST_19MAY_1may19_PHYSICALTEST___1495200146.12.npy')
m2 = np.load('traces/means_PHYTEST_19MAY_2may19_PHYSICALTEST___1495199996.49.npy')
m3 = np.load('traces/means_PHYTEST_19MAY_3may19_PHYSICALTEST___1495199702.16.npy')

ms = [m1,m2,m3]

for m in ms:
    plt.matshow(m)
    plt.colorbar()