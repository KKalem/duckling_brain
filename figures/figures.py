# -*- coding: utf-8 -*-
"""
Created on Sat May 20 18:54:56 2017

@author: ozer
"""

import numpy as np
import matplotlib.pyplot as plt
from mpltools import style

style.use('grayscale')

#perfect_vs_real_sensors
x = np.linspace(0,5,500)
y = np.sin(x)

#%%
y1 = y[:250:50]
x1 = x[:250:50]

y2 = y[250::50]+(np.random.rand(y1.shape[0]) - 0.5)
x2 = x[250::50]+(np.random.rand(y1.shape[0]) - 0.5)

plt.figure()
plt.plot(x,y)
plt.scatter(x1,y1, marker='s', c='black')
plt.scatter(x2,y2, marker='o', c='black')
plt.annotate('Real function',
             xy = (0.75,0.64),
             xytext = (1.12,0.2),
             arrowprops = {'facecolor':'gray', 'width':0.1, 'headwidth':0})

plt.annotate('Perfect measurement',
             xy = (x1[2],y1[2]),
             xytext = (-0.2,1.125),
             arrowprops = {'facecolor':'gray', 'width':0.1, 'headwidth':0, 'shrink':0.2})

plt.annotate('Noisy measurement',
             xy = (x2[2],y2[2]),
             xytext = (4.1,0.4),
             arrowprops = {'facecolor':'gray', 'width':0.1, 'headwidth':0, 'shrink':0.1})

#%%

x1 = x[::50]
y1 = y[::50]+(np.random.rand(y[::50].shape[0]) - 0.5)*0.5

plt.figure()
plt.subplot(121)
plt.plot(x,y, color='gray')
plt.scatter(x1,y1)
plt.plot(x1,y1)

plt.subplot(122)
plt.plot(x,y, color='gray')
plt.scatter(x1,y1)
from sklearn.gaussian_process import GaussianProcessRegressor
import sklearn.gaussian_process.kernels as kernels

gpr = GaussianProcessRegressor(kernel=kernels.RBF(length_scale=0.1))
xx = np.array(x1).reshape(-1,1)
yy = y1

gpr.fit(xx,yy)
means = gpr.predict(np.array(x).reshape(-1,1))
plt.plot(x,means)

#%%
plt.figure()

x1 = [x[100],x[300],x[400]]
y1 = [y[100],y[300],y[400]]
from sklearn.gaussian_process import GaussianProcessRegressor
import sklearn.gaussian_process.kernels as kernels

gpr = GaussianProcessRegressor(kernel=kernels.RBF(length_scale=1))
xx = np.array(x1).reshape(-1,1)
yy = y1

gpr.fit(xx,yy)
means, stds = gpr.predict(np.array(x).reshape(-1,1), return_std=True)
i = 170
i1 = 350
mean, std, xp = means[i],stds[i],x[i]
mean1, std1, xp1 = means[i1],stds[i1],x[i1]

ax1 = plt.subplot(121)
plt.scatter(x1,y1)
plt.plot(x,means)
plt.plot(x,means+stds, '--', color='gray')
plt.plot(x,means-stds, '--', color='gray')
plt.plot([xp,xp],[-1.5,1.5])
plt.plot([xp1,xp1],[-1.5,1.5])
plt.xlabel('Position')
plt.ylabel('Depth')
plt.xlim(0,5)
plt.ylim(-1.5,1.5)

from scipy.stats import norm
plt.subplot(122, sharey = ax1)
xx = np.linspace(0.2, 1.8, 500)
plt.plot(norm.pdf(xx, mean, std),xx)
xx = np.linspace(-1, 1, 500)
plt.plot(norm.pdf(xx, mean1, std1),xx)

plt.xlabel('Probabilty density')
plt.ylim(-1.5,1.5)

plt.tight_layout()

#%%
from sklearn.gaussian_process import GaussianProcessRegressor
import sklearn.gaussian_process.kernels as kernels

x = np.linspace(0,50,1000)
gpr = GaussianProcessRegressor(kernel=kernels.RBF(length_scale=1))

x1 = [x[0],x[100],x[200],x[300],x[400],x[500]]
y1 = [0,1,2,3,4,5]

plt.figure()
for i in range(3,len(x1)+1):

    plt.subplot(3,1,i-2)
    xx = np.array(x1[:i]).reshape(-1,1)
    yy = y1[:i]

    gpr.fit(xx,yy)
    means, stds = gpr.predict(np.array(x).reshape(-1,1), return_std=True)

    plt.scatter(x1[:i],y1[:i])
    plt.plot(x,means)
    plt.plot(x,means+stds, '--', color='gray')
    plt.xlim(0,30)
    plt.ylim(0,5)


#%%
from sklearn.gaussian_process import GaussianProcessRegressor
import sklearn.gaussian_process.kernels as kernels

#y = np.linspace(0,1,x.shape[0])
gpr = GaussianProcessRegressor(kernel=kernels.RBF(length_scale=1))

skips = [50, 74, 100, 150, 200]
plt.figure()

for i,skip in enumerate(skips):
    gpr.fit(np.array(x[::skip]).reshape(-1,1),y[::skip])
    means, stds = gpr.predict(np.array(x).reshape(-1,1), return_std=True)

    plt.subplot(5,1,i+1)
    plt.scatter(x[::skip],y[::skip])
    plt.plot(x,means)
    plt.plot(x,means+stds*10, '--')
    plt.plot(x,means-stds*10, '--')
    plt.xlim(0,5)
#    plt.ylim(-1.5,1.5)
plt.tight_layout()

#%%
#singles
algos = ['Sweeping',
         'Tahirovic',
         'Greedy std=0.13',
         'Greedy std=0.09',
         'Hybrid std=0.13',
         'Hybrid std=0.09']

mses = [4.4,3.50,23.00,14.00,13.02,5.3]
mse_errs = [1,2,8,4,6,1.5]

times = [670,780,290,310,210,495]
time_errs = [12,13,36,53,17,33]

paths = [670,875,360,446,397,660]
path_errs = [30, 53, 118, 92, 83, 75]


plt.figure()
plt.hold(True)
for algo,mse,mse_err,i in zip(algos, mses, mse_errs,range(len(algos))):
    print(algo)
    print(mse,mse_err)
    plt.errorbar(i,mse,xerr=0,yerr=mse_err,fmt='or')
plt.xlim(-1,len(algos))
plt.xticks(range(len(algos)), algos, rotation=17)
plt.title('Mean Square Error Distributions')
plt.hold(False)

plt.figure()
plt.hold(True)
for algo,mse,mse_err,i in zip(algos, times, time_errs,range(len(algos))):
    print(algo)
    print(mse,mse_err)
    plt.errorbar(i,mse,xerr=0,yerr=mse_err,fmt='or')
plt.xlim(-1,len(algos))
plt.xticks(range(len(algos)), algos, rotation=17)
plt.title('Exploration Time Distributions')
plt.hold(False)

plt.figure()
plt.hold(True)
for algo,mse,mse_err,i in zip(algos, paths, path_errs,range(len(algos))):
    print(algo)
    print(mse,mse_err)
    plt.errorbar(i,mse,xerr=0,yerr=mse_err,fmt='or')
plt.xlim(-1,len(algos))
plt.xticks(range(len(algos)), algos, rotation=17)
plt.title('Total Path Length Distributions')
plt.hold(False)


#%%
#multis
algos = ['Tahirovic',
         'Greedy std=0.13',
         'Greedy std=0.09',
         'Hybrid std=0.13',
         'Hybrid std=0.09']

mses = [3.70, 24, 12, 15, 6.5]
mse_errs = [1.2,1.8,8.5,4.9,6.3,1.1]

times = [470, 220, 330, 210, 350]
time_errs = [23,20,52,57,28,41]

paths = [567+560, 216+232, 315+286, 209+194, 257+295]
path_errs = [45, 73, 132, 125, 90, 83]


plt.figure()
plt.hold(True)
for algo,mse,mse_err,i in zip(algos, mses, mse_errs,range(len(algos))):
    print(algo)
    print(mse,mse_err)
    plt.errorbar(i,mse,xerr=0,yerr=mse_err,fmt='or')
plt.xlim(-1,len(algos))
plt.xticks(range(len(algos)), algos, rotation=17)
plt.title('Mean Square Error Distributions')
plt.hold(False)

plt.figure()
plt.hold(True)
for algo,mse,mse_err,i in zip(algos, times, time_errs,range(len(algos))):
    print(algo)
    print(mse,mse_err)
    plt.errorbar(i,mse,xerr=0,yerr=mse_err,fmt='or')
plt.xlim(-1,len(algos))
plt.xticks(range(len(algos)), algos, rotation=17)
plt.title('Exploration Time Distributions')
plt.hold(False)

plt.figure()
plt.hold(True)
for algo,mse,mse_err,i in zip(algos, paths, path_errs,range(len(algos))):
    print(algo)
    print(mse,mse_err)
    plt.errorbar(i,mse,xerr=0,yerr=mse_err,fmt='or')
plt.xlim(-1,len(algos))
plt.xticks(range(len(algos)), algos, rotation=17)
plt.title('Total Path Length Distributions')
plt.hold(False)

#%%
#comm
algos = ['Tahirovic',
         'Greedy std=0.09',
         'Hybrid std=0.09']

mses = [5.33, 14, 12]
mse_errs = [1.6,9.3,7.1]

times = [480, 400, 320]
time_errs = [32,83,45]

paths = [582+570, 390+420, 290+290]
path_errs = [67, 123, 98]


plt.figure()
plt.hold(True)
for algo,mse,mse_err,i in zip(algos, mses, mse_errs,range(len(algos))):
    print(algo)
    print(mse,mse_err)
    plt.errorbar(i,mse,xerr=0,yerr=mse_err,fmt='or')
plt.xlim(-1,len(algos))
plt.xticks(range(len(algos)), algos, rotation=17)
plt.title('Mean Square Error Distributions')
plt.hold(False)

plt.figure()
plt.hold(True)
for algo,mse,mse_err,i in zip(algos, times, time_errs,range(len(algos))):
    print(algo)
    print(mse,mse_err)
    plt.errorbar(i,mse,xerr=0,yerr=mse_err,fmt='or')
plt.xlim(-1,len(algos))
plt.xticks(range(len(algos)), algos, rotation=17)
plt.title('Exploration Time Distributions')
plt.hold(False)

plt.figure()
plt.hold(True)
for algo,mse,mse_err,i in zip(algos, paths, path_errs,range(len(algos))):
    print(algo)
    print(mse,mse_err)
    plt.errorbar(i,mse,xerr=0,yerr=mse_err,fmt='or')
plt.xlim(-1,len(algos))
plt.xticks(range(len(algos)), algos, rotation=17)
plt.title('Total Path Length Distributions')
plt.hold(False)