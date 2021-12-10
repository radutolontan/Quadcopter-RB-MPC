#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Nov  7 06:26:23 2021

@author: raduefb
"""
import numpy as np


def EUL2ROT_MAT(phi,theta,psi):
    # Return the transformation matrix TBE, expressed in terms of the Euler Angles
    R = np.array([[ np.cos(psi)*np.cos(theta) - np.sin(phi)*np.sin(psi)*np.sin(theta),  np.cos(theta)*np.sin(psi) + np.cos(psi)*np.sin(phi)*np.sin(theta), -np.cos(phi)*np.sin(theta)],
                  [-np.cos(phi)*np.sin(psi)                                          ,  np.cos(phi)*np.cos(psi)                                          ,  np.sin(phi)],
                  [np.cos(psi)*np.sin(theta) + np.cos(theta)*np.sin(phi)*np.sin(psi) ,  np.sin(psi)*np.sin(theta) - np.cos(psi)*np.cos(theta)*np.sin(phi),  np.cos(phi)*np.cos(theta)]]).T;
    return R

# def cross2(a,b):
#     c = np.array([[a[1]*b[2]-a[2]*b[1]], 
#                   [a[2]*b[0]-a[0]*b[2]], 
#                   [a[0]*b[1]-a[1]*b[0]]])
#     return c


def vee(A):
    # Return vee-map of matrix
    return np.array([A[2,1],A[0,2],A[1,0]])

def hat(A):
    # Return hat-map of vector
    return np.array([[0    ,-A[2], A[1]],
                     [A[2] ,  0  ,-A[0]],
                     [-A[1], A[0],    0]])

def vec_cross(a,b):
  crossprod = np.array([[a[1]*b[2]-a[2]*b[1]], 
                        [a[2]*b[0]-a[0]*b[2]], 
                        [a[0]*b[1]-a[1]*b[0]]])
  return crossprod