#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov  5 13:57:39 2021

@author: raduefb
"""

import numpy as np
from scipy.integrate import solve_ivp
from quad_properties import quad_MATLAB_sim
from helper_functions import *

def quad_LIN_DE(t, a, f, n1, n2, n3):
    # States description:
    # x[0:9] = R
    # x[9] = p   ;  x[10] = q     ;x[11] = r
    # x[12] = u   ; x[13] = v     ;x[14] = w
    # x[15] = x   ; x[16] = y     ;x[17] = z
    
    # Recompose states
    R = np.reshape(a[0:9],(3,3))
    omega = a[9:12]; Om = hat(omega)
    u, v, w, x, y, z = a[12:18]
    
    # Recompose input moment
    M_xyz = np.array([n1,n2,n3])
    
    g = 9.81 # acceleration due to gravity (9.81 m/s^2)
    e3 = np.array([0,0,1]) # 3e tensor coordinated in the earth frame
    
    # Import appropriate quadcopter physical properties
    m, Ixx, Iyy, Izz, J, _ , _ = quad_MATLAB_sim()
    
    # Create storage for RHS of DE
    # DE is taken from Geometric Tracking Control for Quadrotor UAV by 
    # Lee, Leok, McClamroch

    # Compute rates of change of states
    R_dot = R @ Om
    omega_dot = np.linalg.inv(J) @ (np.reshape(M_xyz,(3,1)) - vec_cross(omega, J@omega))
    v_dot = (f/m)*(R@e3) - g*e3
    x_dot = np.array([u,v,w])
    
    # Repack rates of change of states
    state = np.empty(18)
    state[0:9] = np.reshape(R_dot, (9,))
    state[9:12] = np.reshape(omega_dot, (3,))
    state[12:15] = np.reshape(v_dot, (3,))
    state[15:18] = np.reshape(x_dot, (3,))

    return state

def quad_dynamics_LIN(quadcopter, ref, x_k, f, n_vec):
    # Solve non-linear time-invariant differential equation at the next timestep
    sol = solve_ivp(fun=quad_LIN_DE, t_span=[0, ref.TS], y0=x_k, method='BDF', 
                    t_eval = [ref.TS], args=(f, n_vec[0], n_vec[1], n_vec[2]), rtol=1e-7)
    # Append states at x_k+1 to return vector
    x_k1 = np.reshape(sol.y,18)
    return x_k1

def quad_NL_DE(t, a, f, n1, n2, n3):
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
    state[0] = p + r*(np.cos(phi)*np.tan(theta)) + q*(np.sin(phi)*np.tan(theta))
    state[1] = q*np.cos(phi) - r*np.sin(phi)
    state[2] = r*(np.cos(phi))/(np.cos(theta)) + q*(np.sin(phi))/(np.cos(theta))
    state[3] = ((Iyy-Izz)/Ixx)*r*q + n1/Ixx
    state[4] = ((Izz-Ixx)/Iyy)*p*r + n2/Iyy
    state[5] = ((Ixx-Iyy)/Izz)*p*q + n3/Izz
    state[6] = r*v - q*w - g*np.sin(theta)
    state[7] = p*w - r*u + g*np.sin(phi)*np.cos(theta)
    state[8] = q*u - p*v + g*np.cos(phi)*np.cos(theta) -f/m
    state[9] = w*(np.sin(phi)*np.sin(psi)+np.cos(phi)*np.cos(psi)*np.sin(theta)) - \
               v*(np.cos(phi)*np.sin(psi)-np.cos(psi)*np.sin(phi)*np.sin(theta)) + \
               u*(np.cos(psi)*np.cos(theta))
    state[10] = v*(np.cos(phi)*np.cos(psi)+np.sin(phi)*np.sin(psi)*np.sin(theta)) - \
                w*(np.cos(psi)*np.sin(phi)-np.cos(phi)*np.sin(psi)*np.sin(theta)) + \
                u*(np.cos(theta)*np.sin(psi))
    state[11] = w*(np.cos(phi)*np.cos(theta)) - u*np.sin(theta) + v*(np.cos(theta)*np.sin(phi))
    return state

def quad_dynamics_NL(quadcopter, ref, x_k, f, n_vec):
    # Solve non-linear time-invariant differential equation at the next timestep
    sol = solve_ivp(fun=quad_NL_DE, t_span=[0, ref.TS], y0=x_k, method='RK45', 
                    t_eval = [ref.TS], vectorized=True, args=(f, n_vec[0], n_vec[1], n_vec[2]), rtol=1e-7)
    # Append states at x_k+1 to return vector
    x_k1 = np.reshape(sol.y,12)
    return x_k1




