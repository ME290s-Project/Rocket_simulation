'''
 # @ Author: Zion Deng
 # @ Description: Solver for Nonlinear problem. Adopted from ME231 
 '''

import numpy as np 
import matplotlib.pyplot as plt 
from scipy import interpolate
import pyomo.environ as pyo 

def disc_dyn(x,u,Ts, gamma, Tau, nx):
    """ 
    discretize the system 
    x: state vector,
    u: input, 
    gamma, tau: parameter 
    x1_next = x1 + Ts(sin(x1) + gamma*arctan(x2))
    x2_next = x2 + Ts/ Tau*(x2 - u)
    """
    x_next = np.empty((nx,))
    x_next[0] = x[0] + Ts *(np.sin(x[0]) + gamma *np.arctan(x[1]))
    x_next[1] = x[1] + Ts / Tau*(x[1] - u)
    return x_next 

if __name__ == '__main__': 
    # system parameter 
    tau = 0.2 
    gamma = 10 
    Ts = 0.05 
    N = 120 
    TFinal = Ts * N  # total time 
    # plot the desired trajectory 
    # given desired trajectory 
    tValues    = [0, 3, 3.5, 5.5, 6]
    xDesValues = [0, 0.75*np.pi/4, 0.67*np.pi/4, 1.25*np.pi/4, 1.25*np.pi/4]

    # interpolate step 
    f = interpolate.interp1d(tValues,xDesValues,'linear')
    tGrid = np.linspace(tValues[0],tValues[-1],N+1)
    xDesired = f(tGrid) 

    udotlim = 0.03 
    nx = 2 
    nu = 1 
    model = pyo.ConcreteModel() 
    model.tidx = pyo.Set(initialize = range(0,N+1))
    model.xidx = pyo.Set(initialize = range(0,nx))
    model.uidx = pyo.Set(initialize = range(0,nu))

    # create state and input  variables 
    model.x = pyo.Var(model.xidx, model.tidx)
    model.u = pyo.Var(model.uidx, model.tidx)

    # Objective 
    model.cost = pyo.Objective(expr = sum((model.x[0, t] - xDesired[t])**2 for t in model.tidx if t < N), sense=pyo.minimize)
    model.constraint1 = pyo.Constraint(model.xidx, rule=lambda model, i: model.x[i, 0] == 0.0)
    model.constraint2 = pyo.Constraint(model.tidx, rule=lambda model, t: model.x[0, t+1] == model.x[0, t] + Ts*(pyo.sin(model.x[0, t])+gamma*pyo.atan(model.x[1, t])) 
                                    if t < N else pyo.Constraint.Skip)
    model.constraint3 = pyo.Constraint(model.tidx, rule=lambda model, t: model.x[1, t+1] == model.x[1, t] + (Ts/tau)*(model.x[1, t]-model.u[0, t]) 
                                    if t < N else pyo.Constraint.Skip)
    model.constraint4 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[0, t+1] - model.u[0, t] <= +Ts*udotlim 
                                   if t < N-1 else pyo.Constraint.Skip)
    model.constraint5 = pyo.Constraint(model.tidx, rule=lambda model, t: model.u[0, t+1] - model.u[0, t] >= -Ts*udotlim 
                                    if t < N-1 else pyo.Constraint.Skip)
    model.constraint6 = pyo.Constraint(expr = 0.975*xDesired[N] - model.x[0, N] <= 0.0)
    model.constraint7 = pyo.Constraint(expr = model.x[0, N] - 1.025*xDesired[N] <= 0.0)
    results = pyo.SolverFactory('ipopt').solve(model)
    # plot results
    x1 = [pyo.value(model.x[0,0])]
    x2 = [pyo.value(model.x[1,0])]
    u1 = [pyo.value(model.u[0,0])]

    for t in model.tidx:
        if t < N:
            x1.append(pyo.value(model.x[0,t+1]))
            x2.append(pyo.value(model.x[1,t+1]))
        if t < N-1:
            u1.append(pyo.value(model.u[0,t+1]))

    # Simulate on discretized model using optimal input values
    x_actual = np.zeros((nx, N+1)) # this vector will store the actual x values.
    for t in range(N):
        x_actual[:,t+1] = disc_dyn(x_actual[:,t], u1[t], Ts, gamma, tau, nx) 

    plt.figure()
    plt.plot(tGrid, x_actual[0,:], 'b')
    plt.plot(tGrid, xDesired, 'g')
    plt.plot(tGrid, x1, '--r')
    plt.legend(['Actual', 'Desired', 'Open-Loop'])
    plt.xlabel('Time')
    plt.ylabel('x1 Trajectory')
    plt.show()