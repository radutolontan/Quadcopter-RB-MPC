#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Nov  7 06:37:23 2021

@author: raduefb
"""
import numpy as np
import pyomo.environ as pyo
from helper_functions import transformation_matrix, vee, hat

def att_loop_control (quadcopter, ref, xk, F_vec):
    # quadrotor: quad object of type quad_class
    # ref: instance of trajectory class
    # xk: state vector at current time step
    # Fvec: force vector (x,y,z) components from position controller
    
    # Unpack states
    phi, theta, psi, p, q, r, u, v, w, x, y, z = xk
    
    # Compute unit vector in direction of required force
    b3c = F_vec / np.linalg.norm(F_vec)
    R = transformation_matrix(phi, theta, psi)
    b3 = R[:,2]
    b1d = np.array([1,0,0])
    
    b1c = -np.cross(b3c,np.cross(b3c,b1d.T))
    b1c = b1c / np.linalg.norm(np.cross(b3c,b1d.T))
    
    Rc = np.array([b1c , np.cross(b3c,b1c) , b3c])
    Rd = Rc
    
    # Import reference angular velocity and acceleration
    Omegad = ref.Omega
    dOmegad = ref.dOmega
    
    # Calculate errors in orientation and angular velocity
    err_R = 0.5 * vee((Rd.T)@R - (R.T)@Rd)
    Omega = np.array([p,q,r]).T
    err_Om = Omega - (R.T)@Rd@Omegad
    
    # Compute inputs
    f_T = np.dot(F_vec,b3)
    n_xyz = - (quadcopter.kR)@err_R - (quadcopter.kOm)@err_Om + np.cross(Omega,(quadcopter.J)@Omega) - (quadcopter.J)@(hat(Omega)@(R.T)@Rd@Omegad - (R.T)@Rd@dOmegad)
        
    return f_T, n_xyz

def pos_loop_control(quadrotor, ref, N_MPC, k_cur, x0):
    from pyomo.opt import SolverStatus, TerminationCondition
    # quadrotor: quad object of type quad_class
    # ref: instance of trajectory class
    # N_MPC: horizon of MPC controller
    # k_cur: starting time step
    # x0: initial condition of current optimization problem
    
    # Trim x0 to desired states only (u,v,w,x,y,z)
    x0 = x0[6:12]
    
    # Initialize optimization problem
    model = pyo.ConcreteModel()
    model.N = N_MPC
    model.nx = np.size(quadrotor.A_MPC, 0)
    model.nu = np.size(quadrotor.B_MPC, 1)
    
    # Length of finite optimization problem
    model.tIDX = pyo.Set( initialize= range(model.N+1), ordered=True )  
    model.xIDX = pyo.Set( initialize= range(model.nx), ordered=True )
    model.uIDX = pyo.Set( initialize= range(model.nu), ordered=True )
    
    # Create pyomo objects for the linear system description
    model.A = quadrotor.A_MPC
    model.B = quadrotor.B_MPC

    # Create state and input variables trajectory
    model.x = pyo.Var(model.xIDX, model.tIDX)
    model.u = pyo.Var(model.uIDX, model.tIDX)
    
    # Import tracking trajectory for current time and horizon
    stages = np.linspace(k_cur, k_cur+N_MPC-1, N_MPC)
    ref_x = ref.x(stages)
    
    # Objective
    def stage_cost(model):
        costX = 0.0
        costTerminal = 0.0
        # For all time steps
        for t in model.tIDX:
            # For the three states of interest: x,y,z
            for i in list(range(3,6)):
                if t < model.N-1:
                    costX += (model.x[i, t] - ref_x[i-3,t])**2
        for i in list(range(3,6)): 
            costTerminal += 4 * (model.x[i, model.N-1] - ref_x[i-3, model.N-1])**2
        return costX + costTerminal
    model.cost = pyo.Objective(rule = stage_cost, sense = pyo.minimize)
    
    # System Constraints
    def equality_const_rule(model, i, t):
        return  model.x[i, t+1] - (model.x [i, t] + ref.TS * (sum(model.A[i, j] * model.x[j, t] for j in model.xIDX)
                +sum(model.B[i, j] * model.u[j, t] for j in model.uIDX))) == 0 if t < model.N else pyo.Constraint.Skip
    model.equality_constraints = pyo.Constraint(model.xIDX, model.tIDX, rule=equality_const_rule)
    
    # Initial Conditions Constraints
    model.initial_constraints = pyo.Constraint(model.xIDX, rule=lambda model,i: model.x[i,0]==x0[i])
    
    # Input Constraints
    # model.input_constraints1 = pyo.Constraint(model.tIDX, rule=lambda model,t: np.linalg.norm(model.u[:,t] + 
    #                                                                                           np.array([0,0,quadrotor.mass*quadrotor.g]).T)<=quadrotor.fMax)
    # model.input_constraints2 = pyo.Constraint(model.tIDX, rule=lambda model,t: model.u[2,t] >= - quadrotor.g*quadrotor.mass)
    model.input_constraints1 = pyo.Constraint(model.uIDX, model.tIDX, rule=lambda model,i,t: model.u[i,t]<=quadrotor.uU[i])
    model.input_constraints2 = pyo.Constraint(model.uIDX, model.tIDX, rule=lambda model,i,t: model.u[i,t]>=quadrotor.uL[i])
    
    # Initialize MOSEK solver and solve optimization problem
    solver = pyo.SolverFactory("mosek")
    results = solver.solve(model)
    
    # Check if solver found a feasible, bounded, optimal solution
    if (results.solver.status == SolverStatus.ok) and (results.solver.termination_condition == TerminationCondition.optimal):
        feas = True
        xOpt = np.asarray([[model.x[i,t]() for i in model.xIDX] for t in model.tIDX]).T
        uOpt = np.asarray([model.u[:,t]() for t in model.tIDX]).T
        #print("MPC problem ", k_cur, " solved!")
    else:
        feas = False
        xOpt = 999
        uOpt = 999

      
    return feas, xOpt, uOpt
