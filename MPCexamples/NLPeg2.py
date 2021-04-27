'''
 # @ Author: Zion Deng
 # @ Description: Nonlinear problem example 2 
 minimize: 3*sin(-2pi*z1) + 2z1 +4 + cos(2pi*z2) +z2
 '''

import numpy as np 
import pyomo.environ as pyo 
from numpy import pi 

m = pyo.ConcreteModel() 
m.z1 = pyo.Var(initialize = 0.1) 
m.z2 = pyo.Var(initialize = 0.1) 
m.obj = pyo.Objective(
    expr = 3*pyo.sin(-2*pi*m.z1) + 2*m.z1 + 4 + pyo.cos(2*pi*m.z2) + m.z2
)
m.c1 = pyo.Constraint(
    expr = (-1, m.z1 ,1)
)
m.c2 = pyo.Constraint(
    expr = (-1, m.z2, 1)
)
solver = pyo.SolverFactory('ipopt')
results = solver.solve(m)
print('z1 value: %f, z2 value: %f' %(m.z1(), m.z2()))