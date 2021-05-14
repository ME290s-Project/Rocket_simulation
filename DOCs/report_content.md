- [Introduction](#introduction)
  - [Cost function and constraints](#cost-function-and-constraints)
  - [Loop for MPC solver and simulation](#loop-for-mpc-solver-and-simulation)
- [Other techniques for Landing](#other-techniques-for-landing)
  - [helperOC](#helperoc)
  - [DQN](#dqn)

# MPC

## Cost function and constraints

1. minimum $(x_k - x_{des})^T P (x_k - x_{des})$
2. physical constraints
3. dynamic constraints
4. initial constraints

## Pyomo and ipopt solver 

* Pyomo: programming language which enables fast switching between solvers
* ipopt: solver which can solve Linear problems, quadratic problems and nonlinear problems.


## Loop for MPC solver and simulation

1. initialize
2. Full thrust launch with initial states
3. gliding process with states after launching
4. re-enter process with states after gliding
5. landing process with states after re-entering

## Algorithm


# Other techniques for Landing

* It seems not to be so good to use MPC for landing.
* Trying other algorithms for computing

## helperOC

## DQN

1. action definition:
   1. accelerate
   2. decelerate
   3. left
   4. right
