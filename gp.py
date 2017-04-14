# -*- coding: utf-8 -*-
"""
Created on Wed Mar 22 17:13:33 2017

@author: ozer
"""
import numpy as np

from matplotlib import pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D #needed for '3d'

from sklearn.gaussian_process import GaussianProcessRegressor
import sklearn.gaussian_process.kernels as kernels

import util as u
import config

import time

class GP:
    def __init__(self, length_scale = 5., nu=2.5, noise_level=0.1):
        """
        inits the gpr, no ftting done here
        """
        matern = kernels.Matern(length_scale = length_scale, nu = nu)
        white = kernels.WhiteKernel(noise_level = noise_level)
#        rbf = kernels.RBF(length_scale=length_scale)
        self.gpr = GaussianProcessRegressor(kernel=matern+white, n_restarts_optimizer=15)
        #TODO tune/play with everything above

    def fit(self, measurements):
        """
        only does fitting
        """
        #filter Nones just in case
        measurements = filter(lambda m: not(m[0] is None or m[1] is None or m[2] is None), measurements)
        measurements = np.array(measurements) #make sure its an array now
        X = measurements[:,:2] #positions
        Y = measurements[:,-1] #sonars
        #normalize the depth values
        Y = Y/float(np.max(Y)) - 0.5

        start = time.time()
        self.gpr.fit(X,Y)
        print('fitting time;',time.time()-start)

    def regress(self, targets):
        """
        only does prediction, assuming fitting is done
        """
        filter(lambda t: t is not None, targets)
        means, stds = self.gpr.predict(targets, return_std=True)
        return means, stds


    def show_surface(self, measurements=None, grid_density=20, **kwargs):

        #otherwise, regress for a meshgrid
        #generate a grid for regression
        xmin = kwargs.get('xmin',-config.WINDOW_METERS/2.)
        xmax = kwargs.get('xmax',config.WINDOW_METERS/2.)
        ymin = kwargs.get('ymin',-config.WINDOW_METERS/2.)
        ymax = kwargs.get('ymax',config.WINDOW_METERS/2.)
        xaxis = np.linspace(xmin,xmax, grid_density)
        yaxis = np.linspace(ymin,ymax, grid_density)

        t_xs, t_ys = np.meshgrid(xaxis,yaxis)
        targets = []
        for x in xaxis:
            for y in yaxis:
                targets.append([x,y])
        targets = np.array(targets)

        #do regression over the grid given the data
        start = time.time()
        means,stds = self.regress(targets)
        print('surface regression time;',time.time()-start)

        #reshape and amplify the regression results to be plotted
        means_z = np.reshape(means, [grid_density,grid_density])
        stds_z = np.reshape(stds, [grid_density,grid_density])

        #same colormap for the noise-generated map
        colormap = mpl.colors.ListedColormap(u.load_colormap(config.COLORMAP_FILE))

        #figures and stuff
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        #plot the fancy surfaces
        ax.plot_surface(t_ys, t_xs, means_z,
                        rstride=1, cstride=1,
                        cmap=colormap, linewidth=0, antialiased=True)

        ax.plot_surface(t_ys, t_xs, means_z+stds_z*1.9600,
                        rstride=1, cstride=1, cmap='Greens', alpha=0.5,
                        linewidth=0, antialiased=True)

        if measurements is not None:
            measurements = filter(lambda m: not(m[0] is None or m[1] is None or m[2] is None), measurements)
            measurements = np.array(measurements) #make sure its an array now
            X = measurements[:,:2] #positions
            Y = measurements[:,-1] #sonars
            #normalize the depth values
            Y = Y/float(np.max(Y)) - 0.5
            ax.plot3D(X[:,0], X[:,1], Y, color='red')

        plt.xlabel('x')
        plt.ylabel('y')
        plt.show(block=False)




    def fit_regress(self,targets,measurements):
        """
        Fits the GPR to measurements and returns the regression results for given
        target points
        targets is a list of [x,y]
        measurements is a numpy array or list of [nSamples, (x,y,depth)] shape

        if targets is None or empty, then this method foregoes the target list and
        instead uses a grid of grid x grid targets around the middle of measurements
        and draws a fancy surface plot

        returns (means, stds) of same length as targets
        """
        #filter Nones just in case
        measurements = filter(lambda m: not(m[0] is None or m[1] is None or m[2] is None), measurements)
        measurements = np.array(measurements) #make sure its an array now
        self.fit(measurements)
        if targets is not None and len(targets) > 0:
            #regress for targets if there are any given
            means, stds = self.regress(targets)
        else:
            self.show_surface(measurements)

        return means, stds


    def save_matrix(self, **kwargs):
        """
        save a matrix of depth values of the same size as the generated depthmap in pixels
        for a 1 to 1 pixel error measurement
        """
        suffix = kwargs.get('suffix','_mat')

        xmin = kwargs.get('xmin',-config.WINDOW_METERS/2.)
        xmax = kwargs.get('xmax',config.WINDOW_METERS/2.)
        ymin = kwargs.get('ymin',-config.WINDOW_METERS/2.)
        ymax = kwargs.get('ymax',config.WINDOW_METERS/2.)

        grid_density = kwargs.get('grid_density', config.WINDOW_SIZE)

        xaxis = np.linspace(xmin,xmax, grid_density)
        yaxis = np.linspace(ymin,ymax, grid_density)

        t_xs, t_ys = np.meshgrid(xaxis,yaxis)
        targets = []
        for x in xaxis:
            for y in yaxis:
                targets.append([x,y])
        targets = np.array(targets)

        print('[I] Regressing for saving, this will take a while...')
        means, stds = self.regress(targets)
        means_z = np.reshape(means, [grid_density,grid_density])
        stds_z = np.reshape(stds, [grid_density,grid_density])

        print('[I] Saving matrices')
        np.save(config.TRACE_DIR+'/means'+suffix,means_z)
        np.save(config.TRACE_DIR+'/stds'+suffix,stds_z)

        print('[I] Done')

if __name__=='__main__':
    import util
    m = util.load_trace('0')
    m = np.array(m)
    gp = GP()
    skip = 10
    start = time.time()
    gp.fit(m[::skip])
#    means, stds = gp.regress(targets)
    gp.show_surface(m[::skip])
    t = time.time()-start
    s = len(m[::skip])
    print('total time;',t)
    print('total samples;',s)

#    gp.save_matrix()