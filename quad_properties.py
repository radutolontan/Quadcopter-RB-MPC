#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Nov  7 02:38:02 2021

@author: raduefb
"""
import numpy as np

# Create quadrotor class
class quad_class:
    def __init__(self, mass, Ixx, Iyy, Izz, xL, xU, uL, uU):
        self.mass = mass # Mass (kg)
        self.Ixx = Ixx # mass moment of inertia about x-
        self.Iyy = Iyy # mass moment of inertia about y-
        self.Izz = Izz # mass moment of inertia about z-
        self.J = np.array([[Ixx, 0, 0],
                           [0, Iyy, 0],
                           [0, 0, Izz]])
        
        self.xL = xL # xL < x < xU
        self.xU = xU
        self.uL = uL # uL < u < uU
        self.uU = uU
        
        self.g = 9.81 # (m/s^2)
        
        # Linear system matrices for reduced trajectory system
        A = np.zeros((6,6))
        A[3,0] = 1; A[4,1] = 1; A[5,2] = 1
        self.A_MPC = A
            # for states = u,v,w,x,y,z
        B = np.zeros((6,3))
        B[0,0] = 1/self.mass; B[1,1] = 1/self.mass; B[2,2] = 1/self.mass
        self.B_MPC = B
             # for inputs = fx, fy, fz
   
        # Linear system matrices for full system
        self.A = np.array([[0 , 0 , 0 , 1 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0],
                           [0 , 0 , 0 , 0 , 1 , 0 , 0 , 0 , 0 , 0 , 0 , 0],
                           [0 , 0 , 0 , 0 , 0 , 1 , 0 , 0 , 0 , 0 , 0 , 0],
                           [0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0],
                           [0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0],
                           [0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0],
                           [0 , -self.g , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0],
                           [self.g , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0],
                           [0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0],
                           [0 , 0 , 0 , 0 , 0 , 0 , 1 , 0 , 0 , 0 , 0 , 0],
                           [0 , 0 , 0 , 0 , 0 , 0 , 0 , 1 , 0 , 0 , 0 , 0],
                           [0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 1 , 0 , 0 , 0]])
            # for states = phi,theta,psi,p,q,r,u,v,w,x,y,z
        self.B = np.array([[0 , 0 , 0 , 0],
                           [0 , 0 , 0 , 0],
                           [0 , 0 , 0 , 0],
                           [0 , 1/(self.Ixx) , 0 , 0],
                           [0 , 0 , 1/(self.Iyy) , 0],
                           [0 , 0 , 0 , 1/(self.Izz)],
                           [0 , 0 , 0 , 0],
                           [0 , 0 , 0 , 0],
                           [-1/(self.mass) , 0 , 0 , 0],
                           [0 , 0 , 0 , 0],
                           [0 , 0 , 0 , 0],
                           [0 , 0 , 0 , 0]])
            # for inputs = f, M1, M2, M3
        self.fMax = 35 # Maximum force quad can develop (N)
        self.kR = 200 * np.eye(3)
        self.kOm = 40 * np.eye(3)

def quad_MATLAB_sim():
    # Mass properties
    Ixx = 0.557e-02 # (kg*m^2)
    Iyy = Ixx # (kg*m^2)
    Izz = 1.05e-02 # (kg*m^2)
    m = 0.85 # (kg)
    # State constraints
    xL = np.array([-0.5236, -0.5236, -9e09, -9e09, -9e09, -9e09, -9e09, -9e09, -9e09, -9e09, -9e09, -9e09])
    xU = np.array([0.5236 ,  0.5236,  9e09,  9e09,  9e09,  9e09,  9e09,  9e09,  9e09,  9e09,  9e09,  9e09])
    # Input constraints
    uU = np.array([4, 4, 4])*0.25
    uL = np.array([-4, -4, -4])*0.25
    
    return m, Ixx, Iyy, Izz, xL, xU, uL, uU