#!/usr/bin/env python

#Author: Solomon Jingchun YIN,
#Contact Address: jingchun.yin@live.com
#Date: Apr. 21th, 2015

from numpy import * 
from matplotlib import *
from pylab import *

fs = open('diagnosis_inv_kinematics.txt','r')

aux = [item.split() for item in fs]
fs.close()
A = zeros([ len(aux)-1,len(aux[0]) ])
for i in xrange(1,len(aux)):
    for j in xrange(len(aux[0])):
        A[i-1][j] = float(aux[i][j])
        
#            aux = 'v_x' + '\t' + 'v_y' + '\t' + 'omega'  + '\t' + \
#                'beta_dot_0' + '\t' + 'beta_dot_1' + '\t' + 'phi_dot_0' + '\t' + 'phi_dot_1'+\
#                'beta0' + '\t' + 'beta1' + '\t' + 'phi0' + '\t' + 'phi1'+'\n'

close('all')

rc('font', size=16)

fig_cart = figure()
subplot(3,1,1)
plot(A[:,0], A[:,1],'-')
ylabel(r'$\dot{x}$ (m/s)', fontsize=16)
subplot(3,1,2)
plot(A[:,0], A[:,2],'-')
ylabel(r'$\dot{y}$ (m/s)', fontsize=16)
subplot(3,1,3)
plot(A[:,0], A[:,3],'-')
ylabel(r'$\dot{\theta}$ (rad/s)', fontsize=16)
xlabel(r'$t$ (sec.)', fontsize=16)

for k in xrange(1, 3*1+1):
    subplot(3,1,k)
    grid(True)

fig_cart.suptitle('Cartesian-Space Trajectory Profile', fontsize=30, fontweight='bold')



fig_jnt = figure()

subplot(4,1,1)
plot(A[:,0], A[:,8],'-')
ylabel(r'$\beta_0$ (rad)', fontsize=16)
subplot(4,1,2)
plot(A[:,0], A[:,9],'-')
ylabel(r'$\beta_1$ (rad)', fontsize=16)
subplot(4,1,3)
plot(A[:,0], A[:,4],'-')
ylabel(r'$\dot{\beta}_0$ (rad/s)', fontsize=16)
subplot(4,1,4)
plot(A[:,0], A[:,5],'-')
ylabel(r'$\dot{\beta}_1$ (rad/s', fontsize=16)

xlabel(r'$t$ (sec.)', fontsize=16)

for k in xrange(1, 1*4+1):
    subplot(4,1,k)
    grid(True)

fig_jnt.suptitle('Joint-Space Trajectory for Steering', fontsize=30, fontweight='bold')


fig_spinning = figure()    

subplot(4,1,1)
plot(A[:,0], A[:,10],'-')
ylabel(r'$\phi_0$ (rad)', fontsize=16)
subplot(4,1,2)
plot(A[:,0], A[:,11],'-')
ylabel(r'$\phi_1$ (rad)', fontsize=16)

subplot(4,1,3)
plot(A[:,0], A[:,6],'-')
ylabel(r'$\dot{\phi}_0$ (rad/s', fontsize=16)
subplot(4,1,4)
plot(A[:,0], A[:,7],'-')
ylabel(r'$\dot{\phi}_1$ (rad/s', fontsize=16)

xlabel(r'$t$ (sec.)', fontsize=16)

for k in xrange(1, 1*4+1):
    subplot(4,1,k)
    grid(True)

fig_spinning.suptitle('Joint-Space Trajectory for Rolling', fontsize=30, fontweight='bold')

show()

