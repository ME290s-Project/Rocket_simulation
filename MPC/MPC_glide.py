'''
 # @ Author: Zion Deng
 # @ Description: Try to solve the project with MPC, pyomo 
 Model description: 
 state: x, y, theta, xdot, ydot, thetadot 
 ctrl state: F, delta 
    m=x(7);
    f(1,1) = x(4); % f
    f(2,1) =  x(5); % dy
    f(3,1) = x(6); % d_theta
    f(4,1) = 0.001*(-(rou*Ar*(x(4)^2)/(2*m))-u(2)*(rou*Ag*(x(4)^2)/(2*m))); % df
    f(5,1) = -g-0.001*((rou*Ar*(x(5)^2)/(2*m))-u(2)*(rou*Ag*(x(5)^2)/(2*m))); % ddy
    f(6,1) = 0;
    f(7,1) = 0; % dm
 '''

import numpy as np 
import pyomo.environ as pyo 
import matplotlib.pyplot as plt 

def MPC_solve():
    """ 
    solve with pyomo
    return: feas, xOpt, uOpt 
    """ 
    # Constants
    M = 326956
    ROU = 1.1
    A = 100
    g = 10
    GAMMA = 0.1
    L = 70
    J = 1/2*M*L**2  
    K = GAMMA*ROU*A*g / (2*M)
    ST = 46000
    Ar=100
    Ag=36

    NX = 7  # number of states
    NU = 1  # number of inputs 
    DT = 1 # time interval 
    N = 130 # number of total intervals 
    INITIAL_STATE = [106429, 114856,  66,  889, 571, 0.45, 326956]

    FMAX = 1.1  # the max force that engine can provide 
    DELTAMAX = 0.1
    m = pyo.ConcreteModel()  # pyomo model
    m.tidx = pyo.Set( initialize= range(0,N+1))  # time index 
    m.xidx = pyo.Set( initialize= range(0, NX))  # state index 
    m.uidx = pyo.Set( initialize= range(0, NU))  # input index 

    m.x = pyo.Var(m.xidx, m.tidx)  # model x[i,t]
    m.u = pyo.Var(m.uidx, m.tidx)  # model u[i,t]

    # cost function 
    m.cost = pyo.Objective(
        expr = sum((m.x[2,t] - 100000)for t in range(N-1,N)), 
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
    # xdot += 0.001*(-(rou*Ar*(x(4)^2)/(2*m))-u(2)*(rou*Ag*(x(4)^2)/(2*m)))
    m.dyn_cons4 = pyo.Constraint(
        m.tidx,
        rule = lambda m, t: m.x[3,t+1] == m.x[3,t] + DT*(
            0.001*(-(ROU*Ar*(m.x[3,t]**2)/(2*M))-m.u[0,t]*(ROU*Ag*(m.x[3,t]**2)/(2*M)))
        )
        if t < N else pyo.Constraint.Skip
    )
    # ydot +=  -g-0.001*((rou*Ar*(x(5)^2)/(2*m))-u(2)*(rou*Ag*(x(5)^2)/(2*m)))
    m.dyn_cons5 = pyo.Constraint(
        m.tidx,
        rule = lambda m, t: m.x[4,t+1] == m.x[4,t] + DT*(
             -g-0.001*((ROU*Ar*(m.x[4,t]**2)/(2*M))-m.u[0,t]*(ROU*Ag*(m.x[4,t]**2)/(2*M)))
        )
        if t < N else pyo.Constraint.Skip
    )
    # thetadot += DT*(-F*L*sin(delta) /(2*J))
    m.dyn_cons6 = pyo.Constraint(
        m.tidx,
        rule = lambda m, t: m.x[5,t+1] == 0
        if t < N else pyo.Constraint.Skip
    )
    # mdot += -DT*(F/ST)
    m.dyn_cons7 = pyo.Constraint(
        m.tidx,
        rule = lambda m, t: m.x[6,t+1] == 0
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
    print([i[-1] for i in xOpt])
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

    # plt.figure() 
    # plt.title('Input Figure')
    # plt.subplot(2,1,1)
    # plt.plot(uOpt[0])
    # plt.ylabel('Force')
    # plt.subplot(2,1,2)
    # plt.plot(uOpt[1])
    # plt.ylabel('Delta')

    plt.show() 
    


