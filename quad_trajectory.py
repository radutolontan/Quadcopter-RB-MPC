#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov  9 19:51:06 2021

@author: raduefb
"""

import numpy as np


class trajectory:
    def __init__(self, freq):
        self.freq = freq
        self.lin_velocity = 1.2
        self.TS = 1/freq
        self.R = np.eye(3)
        self.Omega = np.zeros(3)
        self.dOmega = np.zeros(3)
    # For a given array of time steps, return the trajectory preview
    def x(self, k):
        x = np.zeros((3,int(np.size(k)))) 
        x[0,:] = np.array(k) * self.lin_velocity / self.freq
        return x
    
    
# def liniar_traj(k, freq):
#     # k - time steps; freq - frequncy (Hz)
#     # Create storage and set Z- and Y- locations to 0
#     x = np.zeros((3,int(np.size(k))))
#     # X - location increases at constant velocity
#     x[0,:] = np.array(k) * lin_velocity / freq
#     return x

# def corner_traj(k, freq):
#     # k - time step; freq - frequncy (Hz)
#     # Create storage and set X - Z- and Y- locations to 0
#     x = np.zeros((3,int(np.size(k))))
#     i = 0;
#     # X - location increases at constant velocity until it reaches 5
#     while (np.array(k[i]) * lin_velocity / freq) <= 5:
#         x[0,i] = np.array(k[i]) * lin_velocity / freq
#         i+=1
#         if i==np.size(k,0):
#             return x
#     # After it reached 5, set X- to 0 and increase Y- w. same velocity
#     for j in range(i, np.size(k,0)):
#         x[0,j] = 5
#         x[1,j] = (j-i+1) * lin_velocity / freq
#     return x

# def circular_traj(k, freq):
#     # k - time steps; freq - frequncy (Hz)
#     # Create storage and set Z- location to 0
#     x = np.zeros((3,int(np.size(k))))
#     # Set radius of circlular trajectory
#     r = 1 # (m)
#     # X - and Y - location are parametrized
#     x[0,:] =  r * np.cos(np.array(k) * lin_velocity / (freq*r)) 
#     x[1,:] =  r * np.sin(np.array(k) * lin_velocity / (freq*r)) 
#     x[2,:] =  - np.array(k) * (lin_velocity/freq) * 0.07
#     return x

# OMEGA * t = v/r * k/freq