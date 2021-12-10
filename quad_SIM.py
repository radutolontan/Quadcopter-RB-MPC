#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Nov  7 08:42:47 2021

@author: raduefb
"""
# Import Python functions
import numpy as np
import matplotlib.pyplot as plt

# Import User functions for the quadcopter
from helper_functions import *
from quad_controllers import pos_loop_control, att_loop_control
from quad_properties import *
from quad_dynamics import *
from quad_trajectory import *

# Set operating frequencies for the inner and outer loops
freq_inner = 500 # (Hz)
freq_outer = 100 # (Hz)
# Compute the freq. ratio, necessary for calling approrpeiate controller loops
freq_ratio = int(freq_inner/freq_outer)
# Initiate the control_counter to 0 to run the position inner loop
control_counter = freq_ratio

# Set number of time steps
N = 50
# Set MPC horizon
N_MPC = 25

# Select desired quadrotor type and create its object
m, Ixx, Iyy, Izz, J, uL, uU = quad_MATLAB_sim()
quadrotor = quad_class(m, Ixx, Iyy, Izz, J, uL, uU)

# Create desired trajectory class
ref = trajectory(freq_inner)

n_y = []
f_sigma = []

# Allocate storage for states over time
x = np.zeros((18,N+1))
# Set initial conditions on states
phi_0 = 0.12; theta_0 = 0; psi_0 = 0
x[0:9,0] = np.reshape(EUL2ROT_MAT(phi_0,theta_0,psi_0), (9,))
#-----x0---------y0-----------z0---------
x[15,0] = 0 ; x[16,0] = 0 ; x[17,0] = 0

# States description:
# x[0:9,:] = R
# x[9,:] = p   ;  x[10,:] = q     ;x[11,:] = r
# x[12,:] = u   ; x[13,:] = v     ;x[14,:] = w
# x[15,:] = x   ; x[16,:] = y     ;x[17,:] = z

# Run simulation for the desired number of steps
for i in range(0,N):
    
# --------------------------------------------------------------------------- #
# -------------------------- POSITION CONTROL LOOP -------------------------- #
# --------------------------------------------------------------------------- #
    # If freq-ratio cycles elapsed, run the position control loop
    if control_counter==freq_ratio:
        feas, xMPC, uMPC = pos_loop_control(quadrotor, ref, N_MPC, i, x[:,i])
        print("POS loop ", int(i/freq_ratio), " solved!")
        # Extract the first input force vector and account for gravity
        F_xyz = uMPC[:,0] + np.array([0,0,quadrotor.mass*quadrotor.g])
        # Reset the control counter to 1
        control_counter=1
        # If the optimization problem becomes infeasible at any point exit
        if not feas:
            #x = []
            #u = []
            break    
    # Reset the control counter when it's time to run the position controller again
    else:
        control_counter+=1
    
# --------------------------------------------------------------------------- #
# -------------------------- ATTITUDE CONTROL LOOP -------------------------- #
# --------------------------------------------------------------------------- #
    # Run the attitude control loop
    f, n_vec = att_loop_control(quadrotor, ref, x[:,i], F_xyz)
    print("ATT loop ", control_counter - 1, " solved!")
    print("n_y[",i,']= ',n_vec[1])
    n_y.append(n_vec[1])
    f_sigma.append(f)
    
# --------------------------------------------------------------------------- #
# ---------------------------- QUADROTOR DYNAMICS --------------------------- #
# --------------------------------------------------------------------------- #
    # Run the system dynamics to obtain the next state
    x_k1 = quad_dynamics_LIN(quadrotor, ref, x[:,i], f, n_vec)
    x[:,i+1] = x_k1
    print("Dynamics ", control_counter - 1, " OK!")
    print("R[",i,']= ',np.reshape( x_k1[0:9],(3,3) ) )
    print("q[",i,']= ',x_k1[10]*180/3.1415)
    
    
# Result plotting

l = 0.05
fig = plt.figure(figsize=(16,7))

plt.subplot(131)
plt.plot(n_y,'red')
plt.ylabel("Commanded Moment (nY)")

plt.subplot(132)
plt.plot(x[1,:],'green')
plt.ylabel("Theta (\Theta)")

plt.subplot(133)
plt.plot(f_sigma,'blue')
plt.ylabel("Total Force (F_sigma)")

"""
ax = plt.axes(projection='3d')
for i in range (0, 1000, 100):
    R = transformation_matrix(x[0,i], x[1,i], x[2,i])
    CG = np.array([x[9,i],x[10,i],x[11,i]]).T
    P1 = CG + (R.T)@(np.array([l,0,0]).T)
    P2 = CG + (R.T)@(np.array([0,l,0]).T)
    P3 = CG + (R.T)@(np.array([-l,0,0]).T)
    P4 = CG + (R.T)@(np.array([0,-l,0]).T)
    plt.plot((P1[0],P3[0]),(P1[1],P3[1]),(P1[2],P3[2]),'orange')
    plt.plot((P2[0],P4[0]),(P2[1],P4[1]),(P2[2],P4[2]),'green')
"""

        
        
    

