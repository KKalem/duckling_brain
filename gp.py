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
    def __init__(self, length_scale = 1., nu=2.5, noise_level=0.01):
        """
        inits the gpr, no ftting done here
        """
        matern = kernels.Matern(length_scale = length_scale, nu = nu)
        white = kernels.WhiteKernel(noise_level = noise_level)
        rbf = kernels.RBF(length_scale=length_scale)
        self.gpr = GaussianProcessRegressor(kernel=white+matern, n_restarts_optimizer=15)
        #TODO tune/play with everything above

    def fit_regress(self,targets,measurements,
                    grid_density=20,
                    min_width = 50,
                    show_plot=True,
                    **kwargs):
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
        X = measurements[:,:2] #positions
        Y = measurements[:,-1] #sonars
        #normalize the depth values
        Y = Y/np.max(Y) - 0.5

        start = time.time()
        self.gpr.fit(X,Y)
        print('fitting time;',time.time()-start)

        if targets is not None and len(targets) > 0:
            #regress for targets if there are any given
            means, stds = self.gpr.predict(targets, return_std=True)
        else:
            #otherwise, regress for a meshgrid
            #generate a grid for regression
            xx = X[:,0]
            yy = X[:,1]

            xmin,ymin = (np.min(xx), np.min(yy))
            xmax,ymax = (np.max(xx), np.max(yy))
            xmean,ymean = (np.mean(xx), np.mean(yy))

            xdiff,ydiff = (np.abs(xmin-xmax),np.abs(ymin-ymax))
            xrad,yrad = (max(min_width,xdiff),max(min_width,ydiff))

            xaxis = np.linspace(xmean-xrad, xmean+xrad, grid_density)
            yaxis = np.linspace(ymean-yrad, ymean+yrad, grid_density)

            print(xmean-xrad, xmean+xrad)
            print(ymean-yrad, ymean+yrad)

#            min_grid = np.min(X)
#            max_grid = np.max(X)
#            axis = np.linspace(min_grid,max_grid,grid_density)
            t_xs, t_ys = np.meshgrid(xaxis,yaxis)
            targets = []
            for x in xaxis:
                for y in yaxis:
                    targets.append([x,y])
            targets = np.array(targets)

            #do regression over the grid given the data
            start = time.time()
            means,stds = self.gpr.predict(targets, return_std=True)
            print('regression time;',time.time()-start)

            if show_plot:
                #reshape and amplify the regression results to be plotted
                means_z = np.reshape(means, [grid_density,grid_density])
                stds_z = np.reshape(stds, [grid_density,grid_density])

                #same colormap for the noise-generated map
                colormap = mpl.colors.ListedColormap(u.load_colormap(config.COLORMAP_FILE))

                #figures and stuff
                fig = plt.figure()
                ax = fig.add_subplot(111, projection='3d')
                ax.plot3D(X[:,0], X[:,1], Y,'.', color='red')

                #plot the fancy surfaces
                ax.plot_surface(t_ys, t_xs, means_z,
                                rstride=1, cstride=1,
                                cmap=colormap, linewidth=0, antialiased=True)

                ax.plot_surface(t_ys, t_xs, means_z+stds_z*1.9600,
                                rstride=1, cstride=1, cmap='Greens', alpha=0.5,
                                linewidth=0, antialiased=True)
                plt.xlabel('x')
                plt.ylabel('y')
                plt.show(block=False)


        return means, stds


if __name__=='__main__':
    import util
    m = util.load_trace('0')
    m = np.array(m)
#    m = np.array([[0,0,50],[10,10,5]])
    gp = GP()
    skip = 1
    start = time.time()
    means, stds = gp.fit_regress(None,m[::skip], grid_density=30, show_plot=True)
    t = time.time()-start
    s = len(m[::skip])
    print('total time;',t)
    print('total samples;',s)


    """
    times.append(t)
    samples.append(s)
    times = []
    samples = []
    for trials in range(5):
        for i in range(1,20):
            gp = GP(length_scale=1.)
            start = time.time()
            means, stds = gp.fit_regress(None,m0[::i],grid=100, show_plot=False)
            t = time.time()-start
            s = len(m0[::i])
            print('total time;',t)
            print('total samples;',s)
            times.append(t)
            samples.append(s)
    plt.scatter(samples,times)
    plt.xlabel('number of samples for fitting')
    plt.ylabel('total time to fit+regress in seconds')
    plt.show(block=False)
    plt.savefig('samplesxtime.pdf')

    import mes
    plt.ioff()
    m = mes.m
    gp = GP(length_scale=10.)
#    for i in range(1,len(m)):
        m0 = m[:i]
        means, stds = gp.fit_regress(None,m0,grid=30, show_plot=True, i=i)
    """