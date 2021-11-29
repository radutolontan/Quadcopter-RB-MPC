#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov  5 13:57:39 2021

@author: raduefb
"""

import numpy as np
from scipy.integrate import solve_ivp
from quad_properties import quad_MATLAB_sim

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
    state[6] = -g*theta
    state[7] = g*phi
    state[8] = -f/m
    state[9] = u
    state[10] = v
    state[11] = w
    return state

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


def quad_dynamics_LIN(quadcopter, ref, x_k, f, n_vec):
    # Solve non-linear time-invariant differential equation at the next timestep
    sol = solve_ivp(fun=quad_LIN_DE, t_span=[0, ref.TS], y0=x_k, method='RK45', 
                    t_eval = [ref.TS], vectorized=True, args=(f, n_vec[0], n_vec[1], n_vec[2]), rtol=1e-7)
    # Append states at x_k+1 to return vector
    x_k1 = np.reshape(sol.y,12)
    return x_k1




