'''
 # @ Author: Zion Deng
 # @ Description: Try to solve the project with MPC, pyomo 
 Model description: 
 state: x, y, theta, xdot, ydot, thetadot 
 ctrl state: F, delta 
    f(1,1) = x(4);

    f(2,1) = x(5);

    f(3,1) = x(6);

    f(4,1) = (N*Fs*sind(x(3)-u(1))-rou*Ar*(x(4)^2)/2   -u(2)*(rou*Ag*(x(4)^2)/(2)))/x(7);

    f(5,1) = (N*Fs*cosd(x(3)-u(1))-rou*Ar*(x(5)^2)/2    -u(2)*(rou*Ag*(x(5)^2)/(2)))/x(7)-g;

    f(6,1) =-1000*N*Fs*0.5*sind(u(1))/((1/12)*x(7)*L^2);

    f(7,1) =-N*Fs/ST;
 '''

import numpy as np 
import pyomo.environ as pyo 
import matplotlib.pyplot as plt 
from numpy import sin, cos

def MPC_solve():
    """ 
    solve with pyomo
    return: feas, xOpt, uOpt 
    """ 
    # Constants
    M = 5.5e5
    ROU = 1.1
    A = 100
    g = 10
    GAMMA = 0.1
    L = 70
    J = 1/2*M*L**2  
    K = GAMMA*ROU*A*g / (2*M)
    ST = 46000
    Fs=7.6e3
    Ar=100
    Ag=36

    NX = 7  # number of states
    NU = 2  # number of inputs 
    DT = 1 # time interval 
    N = 130 # number of total intervals 
    INITIAL_STATE = [340000, 20000, 0.2, 200, -560, 0, 326956.0]
    DESIRED_STATE=  [360000, 200, 0,  0,0,0, 3e5]

    FMAX = 7e7  # the max force that engine can provide 
    DELTAMAX = 0.1
    m = pyo.ConcreteModel()  # pyomo model
    m.tidx = pyo.Set( initialize= range(0,N+1))  # time index 
    m.xidx = pyo.Set( initialize= range(0, NX))  # state index 
    m.uidx = pyo.Set( initialize= range(0, NU))  # input index 

    m.x = pyo.Var(m.xidx, m.tidx)  # model x[i,t]
    m.u = pyo.Var(m.uidx, m.tidx)  # model u[i,t]

    # cost function 
    m.cost = pyo.Objective(
        expr = sum((m.x[i,t] - DESIRED_STATE[i])**2 for i in m.xidx for t in range(N-10,N)), 
        sense = pyo.minimize 
    )  
    # initial state constraints 
    m.init_cons = pyo.Constraint(
        m.xidx, 
        rule = lambda m, i: m.x[i,0] == INITIAL_STATE[i]
    )  
    # y > 200
    m.height_cons = pyo.Constraint(
        m.tidx,
        rule = lambda m, t: -m.x[1,t] <= 0
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
    # xdot += DT*((N*Fs*sind(x(3)-u(1))-rou*Ar*(x(4)^2)/2   -u(2)*(rou*Ag*(x(4)^2)/(2)))/x(7))
    m.dyn_cons4 = pyo.Constraint(
        m.tidx,
        rule = lambda m, t: m.x[3,t+1] == m.x[3,t] + DT*(
            (3*Fs*pyo.sin(m.x[2,t] - m.u[0,t])-ROU*Ar*(m.x[3,t]**2)/2 -m.u[1,t]*(ROU*Ag*(m.x[3,t]**2)/(2)))/m.x[6,t]
        )
        if t < N else pyo.Constraint.Skip
    )
    # ydot += DT*((N*Fs*cosd(x(3)-u(1))-rou*Ar*(x(5)^2)/2 -u(2)*(rou*Ag*(x(5)^2)/(2)))/x(7)-g)
    m.dyn_cons5 = pyo.Constraint(
        m.tidx,
        rule = lambda m, t: m.x[4,t+1] == m.x[4,t] + DT*(
            (3*Fs*pyo.cos(m.x[2,t] - m.u[0,t])-ROU*Ar*(m.x[4,t]**2)/2 -m.u[1,t]*(ROU*Ag*(m.x[4,t]**2)/(2)))/m.x[6,t]-g
        )
        if t < N else pyo.Constraint.Skip
    )
    # thetadot += DT*(-1000*N*Fs*0.5*sind(u(1))/((1/12)*x(7)*L^2))
    m.dyn_cons6 = pyo.Constraint(
        m.tidx,
        rule = lambda m, t: m.x[5,t+1] == m.x[5,t] + DT*(
            -1000*3*Fs*0.5*pyo.sin(m.u[0,t])/((1/12)*m.x[6,t]*L**2)
        )
        if t < N else pyo.Constraint.Skip
    )
    # mdot += -DT*(F/ST)
    m.dyn_cons7 = pyo.Constraint(
        m.tidx,
        rule = lambda m, t: m.x[6,t+1] == m.x[6,t] + DT*(
            -m.u[0, t] / ST
        )
        if t < N else pyo.Constraint.Skip
    )

    # solve for results 
    results = pyo.SolverFactory('ipopt').solve(m)
    if str(results.solver.termination_condition) == "optimal":
        feas = True
        print('Solution done!')
    else:
        feas = False
        print('Solution infeasible')
    xOpt = np.asarray([[m.x[i,t]() for t in m.tidx] for i in m.xidx])
    uOpt = np.asarray([[m.u[i,t]() for t in m.tidx] for i in m.uidx])
    return feas, xOpt, uOpt


if __name__ == '__main__': 

    feas, xOpt, uOpt = MPC_solve() 
    # plot
    plt.figure() 
    plt.grid()
    x_pos = xOpt[0]
    y_pos = xOpt[1]
    plt.plot(20000,200,'r*')
    plt.plot(x_pos,y_pos,'g-.')
    plt.plot(x_pos[-1],y_pos[-1],'o')
    plt.title('Trajectory')

    plt.figure()
    plt.title('State Figure')
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
    plt.title('Input Figure')
    plt.subplot(2,1,1)
    plt.plot(uOpt[0])
    plt.ylabel('Force')
    plt.subplot(2,1,2)
    plt.plot(uOpt[1])
    plt.ylabel('Delta')

    plt.show() 

