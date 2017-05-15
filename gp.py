# -*- coding: utf-8 -*-
"""
Created on Wed Mar 22 17:13:33 2017

@author: ozer
"""
import numpy as np


from sklearn.gaussian_process import GaussianProcessRegressor
import sklearn.gaussian_process.kernels as kernels

import util as u
import config

import time

if not config.HEADLESS:
    from matplotlib import pyplot as plt
    import matplotlib as mpl
    from mpl_toolkits.mplot3d import Axes3D #needed for '3d'

class GP:
    def __init__(self, **kwargs):
        """
        inits the gpr, no ftting done here
        noise needs to be tuned EXTREMELY CAREFULLY
        if too large, EVERYTHING will be explained by this noise
        and this will cause std dev to be large everywhere
        if too little or none, extremely close samples with different values
        can cause the optimizer to fail miserably, leading to useless models
        """
        kernel = kernels.ConstantKernel(0.5)
#        kernel += kernels.Matern(length_scale = 0.5, nu = 2.5)

        kernel += kernels.RBF(length_scale=5)
        kernel += kernels.WhiteKernel(noise_level = 0.0001)
        self.gpr = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=15)
        #TODO tune/play with everything above

    def fit(self, measurements, **kwargs):
        """
        only does fitting
        """
        #filter Nones just in case
        measurements = filter(lambda m: not(m[0] is None or m[1] is None or m[2] is None), measurements)
        print('[GP] fitting '+str(len(measurements))+' points')
        measurements = np.array(measurements) #make sure its an array now
        X = measurements[:,:2] #positions
        Y = measurements[:,-1] #sonars
        #normalize the depth values
#        Y = Y/float(np.max(Y)) - 0.5
        Y = u.scale_range(Y, 0, 1)

        start = time.time()
        self.gpr.fit(X,Y)
        print('[GP] fitting time; '+str(time.time()-start))


        if kwargs.get('save_surface',False):
            plt.ioff()
            fig, means, stds = self.show_surface(measurements,
                                                 show=False,
                                                 save=True,
                                                 ID=kwargs.get('ID'),
                                                 grid_density=40)

    def regress(self, targets):
        """
        only does prediction, assuming fitting is done
        """
        print('[GP] regressing '+str(len(targets))+' points')
        start = time.time()
        filter(lambda t: t is not None, targets)
        means, stds = self.gpr.predict(targets, return_std=True)
        print('[GP] regressing time; '+str(time.time()-start))
        return means, stds


    def show_surface(self, measurements=None, grid_density=20, **kwargs):

        if not config.SIMULATION:
            return

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
        print('[GP] surface regression time; '+str(time.time()-start))

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
#            Y = Y/float(np.max(Y)) - 0.5
            Y = u.scale_range(Y, 0, 1)
            ax.plot3D(X[:,0], X[:,1], Y, '.', color='red')

        plt.xlabel('x')
        plt.ylabel('y')
        if kwargs.get('show',True):
            plt.show(block=False)

        if kwargs.get('save',False):
            ID = kwargs.get('ID',99)
            name = ID+'_'+str(time.time())
            plt.savefig('etc/surface_'+name+'.png')
            plt.figure()
            plt.matshow(stds_z)
            plt.colorbar()
            plt.savefig('etc/stds_'+name+'.png')
            plt.figure()
            plt.matshow(means_z)
            plt.colorbar()
            plt.savefig('etc/means_'+name+'.png')
            print('saved surfaces')

        return fig, means_z, stds_z




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

        print('[GP] Regressing for saving, this will take a while...')
        means, stds = self.regress(targets)
        means_z = np.reshape(means, [grid_density,grid_density])
        stds_z = np.reshape(stds, [grid_density,grid_density])

        print('[GP] Saving matrices')
        np.save(config.TRACE_DIR+'/means'+suffix,means_z)
        np.save(config.TRACE_DIR+'/stds'+suffix,stds_z)

        print('[GP] Done')

if __name__=='__main__':
    k = -1
    m0 = np.loadtxt('traces/_0_trace_physical')
    m1 = np.loadtxt('traces/_1_trace_physical')
    m = np.vstack([m0,m1])
#    m = m2
#    more_noise = np.random.rand(m[:,2].shape[0])*40
#    m[:,2] += more_noise

#    import Agent
#    a = Agent.Agent('1')
#    a.measurements = list(m1)
#
#    mm = {}
#    m0 = list(m0)
#    for i in range(len(m0)):
#        mm[i] = list(m0[i])
#    a.other_agents = {'1':{'mments':mm}}
#
#    m = a.get_enough_measurements()



    gp = GP()
    skip = 1
    gp.fit(m[::skip])
    fig, means, stds = gp.show_surface(m[::skip], show=True, grid_density=20)


    plt.matshow(stds, origin='lower')
    plt.colorbar()
    plt.show()

    gp.save_matrix(suffix='_May15_FIXED'+config.SUFFIX)

#animation
#    plt.ioff()
#    done = False
#    i0 = 5
#    i1 = 5
#    frame = 0
#    frame_skip = 3
#    m0 = util.load_trace('0')
#    m1 = util.load_trace('1')
#    skip=2
#    while not done:
#        print 'frame',frame
#        m = np.array(m0[:i0]+m1[:i1])[::skip]
#        gp = GP()
#        gp.fit(m)
#        fig, means, stds = gp.show_surface(m, show=False)
#        fig.savefig('animation/'+str(frame)+'.png')
#        if i0+frame_skip < len(m0):
#            i0 += frame_skip
#        if i1+frame_skip < len(m1):
#            i1 += frame_skip
#
#        if i0+frame_skip > len(m0) and i1+frame_skip > len(m1):
#            done = True
#        else:
#            frame += 1



#1D tests and stuff below here


#    m = [[0,10],[10,15],[20,30]]
#    cnt = 10
#    mx = np.linspace(0,50,cnt)
#    my = map(lambda x: np.sin(x), mx)
#    my = [np.random.rand(1) for i in mx]
#    my[10:20] = [10 for i in range(10,20)]


#    trace = np.array(util.load_trace('0'))
#    my = list(trace[:,2])[::4]
#    mx = np.linspace(0,len(my),len(my))
#
#    m = zip(mx,my)
#    m = np.array(m)
#
#    matern = kernels.Matern(length_scale = 20, nu = 2.5)
#    white = kernels.WhiteKernel()
##    rbf = kernels.RBF(length_scale=0.1)
#    kernel = matern+white
#    gpr = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=15)
#    measurements = np.array(m) #make sure its an array now
#    X = measurements[:,0] #positions
#    X = X.reshape(-1,1)
#    Y = measurements[:,1] #sonars
##    Y = Y/float(np.max(Y))
#    Y = u.scale_range(Y, 0, 1)
#    gpr.fit(X,Y)
#
#    xs = np.linspace(-10,len(my)+10,2000)
#    xs = xs.reshape(-1,1)
#    means, stds = gpr.predict(xs,return_std = True)
#    plt.plot(xs,means)
#    plt.scatter(X,Y)
#    plt.plot(xs,1.96*stds+means, color='g')
#    plt.show()