# -*- coding: utf-8 -*-
"""
Created on Mon Mar  9 12:35:32 2017

@author: ozer
Perlin code found on:https://github.com/caseman/noise/blob/master/examples/2dtexture.py
Slightly modified.
"""


from noise import pnoise2
from scipy.ndimage.filters import gaussian_filter
import numpy as np


def generate_map(size, octaves, base=1, sigma = 20):
    """
    Returns a perlin noise generated depth map with values in [0..1]
    Octaves contols how 'islandy' the result is.
    Base 'chooses' a map
    """
    depthmap = np.zeros([size,size])
    freq = 32.0 * octaves

    for y in range(size):
        for x in range(size):
            depthmap[x,y] = int(pnoise2(x / freq, y / freq, octaves, base=base) * 127.0)

    depthmap -= np.min(depthmap)
    depthmap = depthmap/np.max(depthmap)
    gaussian_filter(depthmap, sigma, output=depthmap)

    return depthmap


class Depthmap:
    def __init__(self, px_size, m_size,
                 octaves, px_origin, base, sigma=20,
                 min_depth=0, max_depth=1):
        """
        px_size is the map size in pixels
        m_size is map size in meters
        octaves controls the islandyness of the map
        origins shift the map origin in pixels to these coordinates. px_size/2
        would make it so the middle of the map is the origin
        """
        self.map = generate_map(px_size, octaves, base=base, sigma=sigma)
        self.map = (self.map * (max_depth-min_depth)) + min_depth
        self.px_size = px_size
        self.m_size = m_size
        self.px_origin = px_origin

        #min/max depth this depthmap will produce.
        #min depth can be negative for 'land'
        self.min_depth = min_depth
        self.max_depth = max_depth

        self.ppm = 1.*px_size/m_size

    def get_depth(self, mx, my):
        """
        returns depth given coordiantes in meters
        """
        px = mx*self.ppm + self.px_origin
        py = my*self.ppm + self.px_origin
        d = self.map[int(px),int(py)]
        return d

    def get_depth_px(self, px, py):
        """
        returns depth given coordinates in pixels
        """
        px += self.px_origin
        py += self.px_origin
        d = self.map[int(px),int(py)]
        return d


if __name__=='__main__':
    import matplotlib as mpl
    import matplotlib.pyplot as plt
    import config

    def load_colormap(filename):
        """
        loads a simple matrix file as a colormap into a numpy array
        """
        v = []
        with open(filename) as f:
            for line in f:
                c = [int(part) for part in line.split()]
                v.append(c)
        m = np.array(v)/255.
        return m

    p_size = 800
    m_size = 50
    octaves = 12
    origin = 400
    base = 1
    sigma = 5
    min_depth = -10
    max_depth = 50
    colormap = mpl.colors.ListedColormap(load_colormap(config.COLORMAP_FILE))
    d = Depthmap(p_size, m_size, octaves, origin, base, sigma, min_depth, max_depth)
    plt.matshow(d.map, cmap = colormap, origin='lower')
    plt.colorbar()
