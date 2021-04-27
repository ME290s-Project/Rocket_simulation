'''
 # @ Author: Zion Deng
 # @ Description: Try to solve the project with MPC, pyomo 
 Model description: 
 state: x, y, theta, xdot, ydot, thetadot 
 ctrl state: F, delta 
 xdotdot = F *sin(delta-theta) /m - GAMMA*ROU*A*g/(2*m)*xdot 
 ydotdot=  F*cos(delta-theta) /m - g -GAMMA*ROU*A*g/(2*m)*ydot
 thetadotdot = -F*L*sin(delta) /(2*J)
 '''

import numpy as np 
import pyomo.environ as pyo 
import matplotlib.pyplot as plt 


if __name__ == '__main__': 
    # Constants
    M = 1
    ROU = 1 
    A = 1 
    g = 1
    GAMMA = 0.1
    L = 1
    J = 1/2*M*L**2  

    NX = 6  # number of states
    NU = 2  # number of inputs 
    DT = 0.1 # time interval 
    N = 100 # number of total intervals 
    TFinal = DT * N  # total time 
    INITIAL_STATE = [2.0, 2.0, 0,0,0,0]
    DESIRED_STATE = [8.0, 2.0, 0,0,0,0]
    FMAX = 100  # the max force that engine can provide 
    DELTAMAX = 0.1

    # solve with pyomo 
    m = pyo.ConcreteModel()  # pyomo model
    m.tidx = pyo.Set( initialize= range(0,N+1))  # time index 
    m.xidx = pyo.Set( initialize= range(0, NX))  # state index 
    m.uidx = pyo.Set( initialize= range(0, NU))  # input index 

    m.x = pyo.Var(m.xidx, m.tidx)  # model x[i,t]
    m.u = pyo.Var(m.uidx, m.tidx)  # model u[i,t]

    # cost function 
    m.cost = pyo.Objective(
        expr = sum((m.x[i,N] - DESIRED_STATE[i])**2 for i in m.xidx), 
        sense = pyo.minimize 
    )  
    # initial state constraints 
    m.init_cons = pyo.Constraint(
        m.xidx, 
        rule = lambda m, i: m.x[i,0] == INITIAL_STATE[i]
    )  
    # y > 2 
    m.height_cons = pyo.Constraint(
        m.tidx,
        rule = lambda m, t: -m.x[1,t] <= -2.0 
        if t < N else pyo.Constraint.Skip
    )
    # 0<F<FMAX
    m.u_cons1_1 = pyo.Constraint(
        m.tidx,
        rule = lambda m, t: m.u[0,t] <= FMAX
        if t < N else pyo.Constraint.Skip
    )  
    m.u_cons1_2 = pyo.Constraint(
        m.tidx,
        rule = lambda m, t: -m.u[0,t] <= 0
        if t < N else pyo.Constraint.Skip
    )  
    # -DELTAMAX<delta<DELTAMAX
    m.u_cons2_1 = pyo.Constraint(
        m.tidx,
        rule = lambda m, t: m.u[1,t] <= DELTAMAX
        if t < N else pyo.Constraint.Skip
    )  
    m.u_cons2_2 = pyo.Constraint(
        m.tidx,
        rule = lambda m, t: -m.u[1,t] <= DELTAMAX
        if t < N else pyo.Constraint.Skip
    )  
    # x += xdot * DT 
    m.dyn_cons1 = pyo.Constraint(
        m.tidx,
        rule = lambda m, t: m.x[0,t+1] == m.x[0,t] + DT*m.x[3,t]
        if t < N else pyo.Constraint.Skip
    )
    # y += ydot * DT 
    m.dyn_cons2 = pyo.Constraint(
        m.tidx,
        rule = lambda m, t: m.x[1,t+1] == m.x[1,t] + DT*m.x[4,t]
        if t < N else pyo.Constraint.Skip
    )
    # theta += thetadot * DT 
    m.dyn_cons3 = pyo.Constraint(
        m.tidx,
        rule = lambda m, t: m.x[2,t+1] == m.x[2,t] + DT*m.x[5,t]
        if t < N else pyo.Constraint.Skip
    )
    # xdot += DT*(F*sin(delta-theta) /M - GAMMA*ROU*A*g/(2*m)*xdot)
    m.dyn_cons4 = pyo.Constraint(
        m.tidx,
        rule = lambda m, t: m.x[3,t+1] == m.x[3,t] + DT*(
            m.u[0,t]*pyo.sin(m.u[1,t] - m.x[2,t]) / M 
        )
        if t < N else pyo.Constraint.Skip
    )
    # ydot += F*cos(delta-theta) /m - g -GAMMA*ROU*A*g/(2*m)*ydot
    m.dyn_cons5 = pyo.Constraint(
        m.tidx,
        rule = lambda m, t: m.x[4,t+1] == m.x[4,t] + DT*(
            m.u[0,t]*pyo.cos(m.u[1,t] - m.x[2,t]) / M -g
        )
        if t < N else pyo.Constraint.Skip
    )
    # thetadot += -F*L*sin(delta) /(2*J)
    m.dyn_cons6 = pyo.Constraint(
        m.tidx,
        rule = lambda m, t: m.x[5,t+1] == m.x[5,t] + DT*(
            -m.u[0, t]*L*pyo.sin(m.u[1,t]) / (2*J)
        )
        if t < N else pyo.Constraint.Skip
    )

    results = pyo.SolverFactory('ipopt').solve(m)
    if str(results.solver.termination_condition) == "optimal":
        feas = True
        print('Solution done! ')
    else:
        feas = False
    xOpt = np.asarray([[m.x[i,t]() for t in m.tidx] for i in m.xidx])
    uOpt = np.asarray([[m.u[i,t]() for t in m.tidx] for i in m.uidx])
    x_pos = xOpt[0]
    y_pos = xOpt[1]

    plt.figure()
    plt.subplot(2,2,1)
    plt.plot(xOpt[2])
    plt.ylabel('theta')
    plt.subplot(2,2,2)
    plt.plot(xOpt[3])
    plt.ylabel('x_dot')
    plt.subplot(2,2,3)
    plt.plot(xOpt[4])
    plt.ylabel('y_dot')
    plt.subplot(2,2,4)
    plt.plot(xOpt[5])
    plt.ylabel('theta_dot')

    plt.figure() 
    plt.subplot(2,1,1)
    plt.plot(uOpt[0])
    plt.ylabel('Force')
    plt.subplot(2,1,2)
    plt.plot(uOpt[1])
    plt.ylabel('Delta')

    plt.figure() 
    plt.plot(2,2,'r*')
    plt.plot(x_pos,y_pos,'g-.')
    plt.plot(x_pos[-1],y_pos[-1],'o')
    plt.show() 
