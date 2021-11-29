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
N = 400
# Set MPC horizon
N_MPC = 25

# Select desired quadrotor type and create its object
m, Ixx, Iyy, Izz, xL, xU, uL, uU = quad_MATLAB_sim()
quadrotor = quad_class(m, Ixx, Iyy, Izz, xL, xU, uL, uU)

# Create desired trajectory class
ref = trajectory(freq_outer)

# Allocate storage for states over time
x = np.zeros((12,N+1))

# Set initial conditions on states
x[9,0] = 0 ; x[10,0] = -0.2 ; x[11,0] = 0.15

# States description:
# x[0,:] = phi ;  x[1,:] = theta ; x[2,:] = psi 
# x[3,:] = p   ;  x[4,:] = q     ; x[5,:] = r
# x[6,:] = u   ;  x[7,:] = v     ; x[8,:] = w
# x[9,:] = x   ; x[10,:] = y     ;x[11,:] = z

# Run simulation for the desired number of steps
for i in range(0,N):
    
# --------------------------------------------------------------------------- #
# -------------------------- POSITION CONTROL LOOP -------------------------- #
# --------------------------------------------------------------------------- #
    # If freq-ratio cycles elapsed, run the position control loop
    if control_counter==freq_ratio:
        feas, xMPC, uMPC = pos_loop_control(quadrotor, ref, N_MPC, i, x[:,i])
        print("POS loop ", int(i/freq_ratio), " solved!")
        # Extract the first input force vector
        F_xyz = uMPC[:,0]
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
    
# --------------------------------------------------------------------------- #
# ---------------------------- QUADROTOR DYNAMICS --------------------------- #
# --------------------------------------------------------------------------- #
    # Run the system dynamics to obtain the next state
    x_k1 = quad_dynamics_LIN(quadrotor, ref, x[:,i], f, n_vec)
    x[:,i+1] = x_k1
    print("Dynamics ", control_counter - 1, " OK!")

        
        
    

