#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 19 15:10:47 2021

@author: raduefb
"""

import numpy as np
import matplotlib.pyplot as plt

from helper_functions import *
from quad_dynamics import *

"""
l = 0.1

theta = 3.1415/6;
phi = 0;
psi = 0;

R = transformation_matrix(phi,theta,psi)

CG = np.array([0,0,0]).T

P1 = CG + (R.T)@(np.array([l,0,0]).T)
P2 = CG + (R.T)@(np.array([0,l,0]).T)
P3 = CG + (R.T)@(np.array([-l,0,0]).T)
P4 = CG + (R.T)@(np.array([0,-l,0]).T)



fig = plt.figure()
ax = plt.axes(projection='3d')
plt.plot((P1[0],P3[0]),(P1[1],P3[1]),(P1[2],P3[2]),'orange')
plt.plot((P2[0],P4[0]),(P2[1],P4[1]),(P2[2],P4[2]),'green')
"""
def quad_LIN_DE(t, a, f, n1, n2, n3):
    # a[0] - phi;  a[1] - theta;  a[2] - psi
    # a[3] - p  ;  a[4] - q    ;  a[5] - r
    # a[6] - u  ;  a[7] - v    ;  a[8] - w 
    # a[9] - x  ;  a[10] -y    ;  a[11] -z 
    phi, theta, psi, p, q, r, u, v, w, x, y, z = a
    g = 9.81 # acceleration due to gravity (9.81 m/s^2)
    
    # Import appropriate quadcopter physical properties
    m, Ixx, Iyy, Izz, _ , _ , _ , _ = quad_MATLAB_sim()
    
    # Create storage for RHS of DE
    # DE is taken from F. Sabatino, "Quadrotor Control: modeling, nonlinear
    # control design, and simulation", June 2015, KTH Sweden
    state = np.empty(12)
    state[0] = p 
    state[1] = q
    state[2] = r
    state[3] = n1/Ixx
    state[4] = n2/Iyy
    state[5] = n3/Izz
    state[6] = g*theta
    state[7] = -g*phi
    state[8] = f/m - g
    state[9] = u
    state[10] = v
    state[11] = w
    return state

x0 = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1])
f = 0.85*9.81# 0.85 * 9.81
n_vec = np.array([0,10,0])

sol = solve_ivp(fun=quad_LIN_DE, t_span=[0, 10], y0=x0, method='RK45', 
                   vectorized=True, args=(f, n_vec[0], n_vec[1], n_vec[2]), rtol=1e-7)

z = sol.y[11,:]
w = sol.y[8, :]
q = sol.y[4, :]
theta = sol.y[1, :]

print("z: ",z)
print("w: ",w)
print("q: ",q)
print("theta: ",theta)
#x_k1 = np.reshape(sol.y,12)



