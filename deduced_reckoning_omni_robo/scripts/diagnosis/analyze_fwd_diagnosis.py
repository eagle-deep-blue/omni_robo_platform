#!/usr/bin/env python

#Author: Solomon Jingchun YIN,
#Contact Address: jingchun.yin@live.com
#Date: Apr. 21th, 2015

from numpy import * 
from matplotlib import *
from pylab import *

fs = open('diagnosis_fwd_kinematics.txt','r')
aux = [item.split() for item in fs]
fs.close()

A = zeros([ len(aux)-1,len(aux[0]) ])
for i in xrange(1,len(aux)):
    for j in xrange(len(aux[0])):
        A[i-1][j] = float(aux[i][j])

close('all')
fig_cart = figure()
subplot(3,1,1)
plot(A[:,0], A[:,1],'r-')
ylabel(r'$x$ (m)')
subplot(3,1,2)
plot(A[:,0], A[:,2],'r-')
ylabel(r'$y$ (m)')
subplot(3,1,3)
plot(A[:,0], A[:,3],'r-')
ylabel(r'$\theta$ (rad)')
fig_cart.suptitle('Estimated Cartesian-Space Trajectory', fontsize=20, fontweight='bold')
for k in xrange(1, 3*1+1):
    subplot(3,1,k)
    grid(True)        
        
fig = figure()
plot(A[:,1],A[:,2],'rx-')        
grid(True)
fig.suptitle('Estimated Robot Route', fontsize=20, fontweight = 'bold')
show()
