'''
 # @ Author: Zion Deng
 # @ Description: A rocket class for landing 
 '''
import numpy as np  
from numpy import sin, cos, pi 
import matplotlib.pyplot as plt 

class Rocket(): 
    ''' 
    state: x, y, theta, xdot, ydot, thetadot 
    ctrl state: F, delta 
    xdotdot = F *sin(delta-theta) /m - GAMMA*ROU*A*g/(2*m)*xdot 
    ydotdot=  F*cos(delta-theta) /m - g -GAMMA*ROU*A*g/(2*m)*ydot
    thetadotdot = -F*L*sin(delta) /(2*J) 
    ''' 


    def __init__(self):
        self.BODY_LENGTH = 70 
        self.WING_LENGTH = 10
        # self.wing_angle = 0 # the angle of the wing
        # 6 states below
        self.x = 20000 
        self.y = 200
        self.theta = 0  # the angle of the body -> theta 
        self.x_dot=  0 
        self.y_dot= 0 
        self.theta_dot= 0 
        # 2 input below
        self.engine_force = 0 
        self.force_angle = 0 # the angle of the force from the body
        self.history = [] # list of visited positions 

    def react(self,action):
        """ 
        action0: accelerate, no turning, action1: decelerate, no turning
        action2: accelerate, turn left, action3: decelerate, turn left
        action4: accelerate, turn right, action5: decelerate, turn right
        return: x, y, theta 
        """
        DT = 1  # the time interval

        M = 5.5e5
        ROU = 1.1
        A = 100
        g = 10
        GAMMA = 0.1
        L = 70
        J = 1/2*M*L**2  
        K = GAMMA*ROU*A*g / (2*M)
        fire_reaction = action % 2  # 0: accelerate, 1: decelerate
        angle_reaction = action // 2  # 0: no turning, 1: turn left, 2: turn right
        if fire_reaction ==0: 
            self.engine_force += 1e7 if self.engine_force < 7e7 else 0 
        else:
            self.engine_force -= 1e7 if self.engine_force > 0 else 0 
        
        if angle_reaction == 1:
            self.force_angle = 0.05 
        elif angle_reaction == 2: 
            self.force_angle = -0.05 
        else: 
            self.force_angle = 0
        # dynamic calculation 

        # x += xdot * DT 
        # y += ydot * DT 
        # theta += thetadot * DT 
        # xdot += DT*(F*sin(delta+theta) /M - K*xdot **2)
        # ydot += DT*(F*cos(delta+theta) /m - g -K*ydot**2)
        # thetadot += DT*(-F*L*sin(delta) /(2*J))
        self.x += DT * self.x_dot
        self.y += DT * self.y_dot
        self.theta += DT * self.theta_dot 
        self.x_dot += DT * (self.engine_force*sin(self.theta + self.force_angle) /M - K*self.x_dot **2)
        self.y_dot += DT * (self.engine_force*cos(self.theta + self.force_angle) /M - K*self.y_dot **2 -g)
        self.theta_dot += DT * (-self.engine_force*L *sin(self.force_angle) / (2*J))
        if self.theta < -pi: 
            self.theta += 2*pi 
        elif self.theta > pi: 
            self.theta -= 2*pi  
        self.history.append(self.get_state())
        return self.get_state()

    def get_state(self):
        return [self.x, self.y, self.theta]




