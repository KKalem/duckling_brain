# -*- coding: utf-8 -*-
"""
Created on Wed Mar 22 17:13:33 2017

@author: ozer
"""
import numpy as np

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, WhiteKernel
import sklearn.gaussian_process.kernels as kernels

import util as u
import config

class GP:
    def __init__(self, measurements, length_scale = 1., nu=5./2, noise_level=1e-10):
        """
        measurements is a numpy array of [nSamples, (x,y,depth)] shape
        no copy of this is made, mutation of it will affect the results
        """
        self.X = measurements[:,:2]
        self.Y = measurements[:,-1]
        matern = kernels.Matern(length_scale = length_scale, nu = nu)
#        white = kernels.WhiteKernel(noise_level = noise_level)
#        kernel = matern+white
        self.gpr = GaussianProcessRegressor(kernel=matern, n_restarts_optimizer=15)
        #TODO tune/play with everything above

    def fit_regress(self,targets,measurements=None):
        """
        Re-fits the GPR to new points and returns the regression results for given
        target points
        targets is a list of [x,y]
        measurements is optional. If not given, the measurement array of init time
        is used instead.

        returns [means, stds]
        """
        if measurements is not None:
            X = measurements[:,:2]
            Y = measurements[:,-1]
        else:
            X = self.X
            Y = self.Y

        self.gpr.fit(X,Y)

        means, stds = self.gpr.predict(targets, return_std=True)
        return means, stds




if __name__=='__main__':
    measurements = []
    xs = np.linspace(-10,10,10)
    ys = np.linspace(-10,10,10)
    for x in xs:
        for y in ys:
            z = x**2+y**3 + np.random.rand()*500
            measurements.append([x,y,z])
    measurements = np.array(measurements)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot3D(measurements[:,0], measurements[:,1], measurements[:,2],'.', color='red')
#    plt.show()

    num_targets = 30
    axis = np.linspace(-10,10,num_targets)
    t_xs, t_ys = np.meshgrid(np.linspace(-10,10,num_targets),np.linspace(-10,10,num_targets))
    targets = []
    for x in axis:
        for y in axis:
            targets.append([x,y])
    targets = np.array(targets)

    gp = GP(measurements, length_scale = 10)
    means,stds = gp.fit_regress(targets)

    means_z = np.reshape(means, [num_targets,num_targets])
    stds_z = np.reshape(stds, [num_targets,num_targets])
    stds_z = stds_z**5

    import matplotlib as mpl
    colormap = mpl.colors.ListedColormap(u.load_colormap(config.COLORMAP_FILE))

    ax.plot_surface(t_ys, t_xs, means_z, rstride=1, cstride=1, cmap=colormap,
                    linewidth=0, antialiased=True)
    ax.plot_surface(t_ys, t_xs, means_z+stds_z+100, rstride=1, cstride=1, cmap='Accent', alpha=0.8,
                    linewidth=0, antialiased=True)



"""
#  Inputs
X = np.array([[1,1],[1,4],[3,4],[5,2],[2,2],[8,4]])
# Observations
Y = np.array([ 10,  15,   17,   6,    9,     15])
# Input space
grid = []
for x in np.arange(0,10,0.5):
    for y in np.arange(0,10,0.5):
        grid.append([x,y])
x=np.array(grid)

kernel = RBF([5,5], (1e-2, 1e2))
gp = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=15)
gp.fit(X, Y)
y_pred, MSE = gp.predict(x, return_std=True)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
Xp, Yp = np.meshgrid(np.arange(0,10,0.5), np.arange(0,10,0.5))
Zp = np.reshape(y_pred,[20,20])
std = np.reshape(MSE,[20,20])
std_low = Zp - std
std_hi = Zp + std

surf = ax.plot_surface(Xp, Yp, Zp, rstride=1, cstride=1, cmap='hot',
linewidth=0, antialiased=False)
surf_low = ax.plot_surface(Xp, Yp, std_low, rstride=1, cstride=1, cmap='Blues',
linewidth=0, antialiased=False, alpha=0.5)
surf_hi = ax.plot_surface(Xp, Yp, std_hi, rstride=1, cstride=1, cmap='Blues',
linewidth=0, antialiased=False, alpha=0.5)

xs = X[:,0]
ys = X[:,1]
zs = Y
ax.plot3D(xs,ys,zs,'.',color='red')
plt.show()

"""