# Quadcopter-RB-MPC

This is a simple simulation of a rigid body drone system using an MPC position controller and a PD attitude controller. Both controllers use linear dynamics and the actual dynamic solver can be changed between linear and nonlinear by changing the function call in line 85 of quad_sim.py (either quad_dynamics_LIN or quad_dynamics_NL).

To run the simulation, open quad_sim.py. Currently, in the absence of mini snap trajectories, the only trajectory simulated is linear.
