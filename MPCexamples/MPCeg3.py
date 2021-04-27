'''
 # @ Author: Zion Deng
 # @ Description: MPC method adopted from ME231 class, but for linear problem
 '''

import numpy as np
import scipy.signal
import scipy.linalg
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import polytope as pt
from numpy import mod, pi 
import pyomo.environ as pyo 

def dlqr(A,B,Q,R): 
    # solve Discrete Algebraic Riccatti equation  
    P = scipy.linalg.solve_discrete_are(A, B, Q, R)

    # compute the LQR gain
    K = scipy.linalg.inv(B.T @ P @ B + R) @ (B.T @ P @ A)

    # stability check 
    # eigVals, eigVecs = scipy.linalg.eig(A - B @ K)
    return K, P


def minkowski_sum(X, Y):
    """ 
    Minkowski sum between two polytopes based on 
    vertex enumeration. So, it's not fast for the
    high dimensional polytopes with lots of vertices. """
    V_sum = []
    if isinstance(X, pt.Polytope):
        V1 = pt.extreme(X)
    else:
        # assuming vertices are in (N x d) shape. N # of vertices, d dimension
        V1 = X
        
    if isinstance(Y, pt.Polytope):
        V2 = pt.extreme(Y)
    else:
        V2 = Y

    for i in range(V1.shape[0]):
        for j in range(V2.shape[0]):
            V_sum.append(V1[i,:] + V2[j,:])
    return pt.qhull(np.asarray(V_sum))

def precursor(Sset, A, Uset=pt.Polytope(), B=np.array([])):
    """ The Pre(S) is the set of states which evolve into the target set S in one time step. """
    if not B.any():
        return pt.Polytope(Sset.A @ A, Sset.b)  # (HA, b)
    else:
        tmp  = minkowski_sum( Sset, pt.extreme(Uset) @ -B.T )
    return pt.Polytope(tmp.A @ A, tmp.b)

def Oinf(A, Xset):
    Omega = Xset
    k = 0
    Omegap = precursor(Omega, A).intersect(Omega)
    while not Omegap == Omega:
        k += 1
        Omega = Omegap
        Omegap = pt.reduce(precursor(Omega, A).intersect(Omega))
    return Omegap

def solve_cftoc(A, B, P, Q, R, N, x0, x1U, x2U, uL, uU, bf, Af):
    """ 
    A,B,P,Q,R:already defined
    N: Horizon 
    x0: initial state 
    x1U: x1 bound 
    uL, uU: input up and low bound 
    bf, Af: Af @ xf <= bf 
     """
    model = pyo.ConcreteModel()
    model.N = N
    model.nx = np.size(A, 0)
    model.nu = np.size(B, 1)
    model.nf = np.size(Af, 0)
    
    # length of finite optimization problem:
    model.tIDX = pyo.Set( initialize= range(model.N+1), ordered=True )  
    model.xIDX = pyo.Set( initialize= range(model.nx), ordered=True )
    model.uIDX = pyo.Set( initialize= range(model.nu), ordered=True )
    
    model.nfIDX = pyo.Set( initialize= range(model.nf), ordered=True )
    
    # Create state and input variables trajectory:
    model.x = pyo.Var(model.xIDX, model.tIDX)
    model.u = pyo.Var(model.uIDX, model.tIDX, bounds=(uL,uU))
    
    #Objective:
    def objective_rule(model):
        costX = 0.0
        costU = 0.0
        costTerminal = 0.0
        for t in model.tIDX:
            for i in model.xIDX:
                for j in model.xIDX:
                    if t < model.N:
                        costX += model.x[i, t] * Q[i, j] * model.x[j, t] 
        for t in model.tIDX:
            for i in model.uIDX:
                for j in model.uIDX:
                    if t < model.N:
                        costU += model.u[i, t] * R[i, j] * model.u[j, t]
        for i in model.xIDX:
            for j in model.xIDX:               
                costTerminal += model.x[i, model.N] * P[i, j] * model.x[j, model.N]
        return costX + costU + costTerminal
    
    model.cost = pyo.Objective(rule = objective_rule, sense = pyo.minimize)
    
    # Constraints:
    def equality_const_rule(model, i, t):
        return model.x[i, t+1] - (sum(A[i, j] * model.x[j, t] for j in model.xIDX)
                               +  sum(B[i, j] * model.u[j, t] for j in model.uIDX) ) == 0.0 if t < model.N else pyo.Constraint.Skip

    model.equality_constraints = pyo.Constraint(model.xIDX, model.tIDX, rule=equality_const_rule)
    model.init_const1 = pyo.Constraint(expr = model.x[0, 0] == x0[0])
    model.init_const2 = pyo.Constraint(expr = model.x[1, 0] == x0[1])
    model.state_limit1 = pyo.Constraint(model.tIDX, rule=lambda model, t: model.x[0, t] <= x1U
                                   if t < N else pyo.Constraint.Skip)
    model.state_limit2 = pyo.Constraint(model.tIDX, rule=lambda model, t: -x1U <= model.x[0, t]
                                    if t < N else pyo.Constraint.Skip)
    model.state_limit3 = pyo.Constraint(model.tIDX, rule=lambda model, t: model.x[1, t] <= x2U
                                   if t < N else pyo.Constraint.Skip)
    model.state_limit4 = pyo.Constraint(model.tIDX, rule=lambda model, t: -x2U <= model.x[1, t]
                                    if t < N else pyo.Constraint.Skip)
    
    def final_const_rule(model, i):
        return sum(Af[i, j] * model.x[j, model.N] for j in model.xIDX) <= bf[i] 
    
    model.final_const = pyo.Constraint(model.nfIDX, rule=final_const_rule)
    
    solver = pyo.SolverFactory('ipopt')
    results = solver.solve(model)
    
    if str(results.solver.termination_condition) == "optimal":
        feas = True
    else:
        feas = False
            
    xOpt = np.asarray([[model.x[i,t]() for i in model.xIDX] for t in model.tIDX]).T
    uOpt = np.asarray([model.u[:,t]() for t in model.tIDX]).T
    
    JOpt = model.cost()
      
    return [model, feas, xOpt, uOpt, JOpt]



if __name__ == '__main__':
    Ts = 0.1       # Ts is the discrete sample-time. 

    Ac = np.array([[0, 1],
                [2.5, -0.05]])
    Bc = np.array([[0],
                [2.5]])

    # We don't need C and D here. This is just for calling the function
    Cc = np.zeros((1,2))
    Dc = np.zeros((1,1))
    system = (Ac, Bc, Cc, Dc)
    A, B, C, D, dt = scipy.signal.cont2discrete(system, Ts)

    nx = np.size(A,0)  # number of states
    nu = np.size(B,1)  # number of inputs

    Q = np.eye(2)
    R = np.array([1])

    uU = np.pi/10
    uX1 = np.pi/4
    uX2 = np.pi/2
    Finf, Pinf = dlqr(A, B, Q, R)

    Acl = A - B @ Finf

    x1U = np.pi/4
    x2U = np.pi/2
    uU = np.pi/10

    # constraint sets represented as polyhedra
    # state constraint
    X = pt.Polytope(np.array([[1.0, 0], 
                            [0, 1.0],
                            [-1, 0],
                            [0, -1]]), 
                    np.array([[x1U], 
                            [x2U],
                            [x1U],
                            [x2U]])) 
    # input constraint
    U = pt.Polytope(np.array([1, -1]).reshape(2,1),
                    np.array([uU, uU]).reshape(2,1))

    # remeber to convet input constraits in state constraints
    S = X.intersect(pt.Polytope(U.A @ -Finf, U.b))

    O_inf = Oinf(Acl, S)   # maximal positive invariant set


    Xf = O_inf
    Af = Xf.A
    bf = Xf.b

    Q = np.eye(2)
    R = np.array([1]).reshape(1,1)
    P = Pinf  # play here with different P
    N = 3
    x1U = np.pi/4
    x2U = np.pi/2
    uU = np.pi/10
    x0 = np.array([-0.5, 1])

    [model, feas, xOpt, uOpt, JOpt] = solve_cftoc(A, B, P, Q, R, N, x0, x1U, x2U, -uU, uU, bf, Af)

    # Setup the simulation with MPC controller

    nx = np.size(A, 0)
    nu = np.size(B, 1)

    M = 25   # Simulation steps
    xOpt = np.zeros((nx, M+1))
    uOpt = np.zeros((nu, M))
    xOpt[:, 0] = x0.reshape(nx, )

    xPred = np.zeros((nx, N+1, M))
    predErr = np.zeros((nx, M-N+1))

    feas = np.zeros((M, ), dtype=bool)
    xN = np.zeros((nx,1))

    fig = plt.figure(figsize=(9, 6))
    for t in range(M):
        [model, feas[t], x, u, J] = solve_cftoc(A, B, P, Q, R, N, xOpt[:, t], x1U, x2U, -uU, uU, bf, Af)
        
        if not feas[t]:
            xOpt = []
            uOpt = []
            predErr = []
            break
        # Save open loop predictions
        xPred[:, :, t] = x

        # Save closed loop trajectory
        # Note that the second column of x represents the optimal closed loop state
        xOpt[:, t+1] = x[:, 1]
        uOpt[:, t] = u[:, 0].reshape(nu, )

        # Plot Open Loop
        line1 = plt.plot(x[0,:], x[1,:], 'r--')

    # Plot Closed Loop
    line2 = plt.plot(xOpt[0, :], xOpt[1, :], 'bo-')
    plt.legend([line1[0], line2[0]], ['Open-loop', 'Closed-loop'])
    plt.xlabel('x1')
    plt.ylabel('x2')
    plt.axis('equal')
    plt.show()
