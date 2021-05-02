'''
 # @ Author: Zion Deng
 # @ Description: Reinforcement environment for rocket landing 
 '''

from matplotlib import pyplot as plt
from Rocket import Rocket 

class RocketEnv(): 
    def __init__(self):
        self.rocket = Rocket() 
        self.REWARD_REACH = 100
        self.REWARD_BOUND = -10
        self.REWARD_CRASH = -10
        self.REWARD_UNSAFE = 10
        self.BOUNDS = [100000,40000]  # tune according to the environment
        self.SAFE_REGION = [55000,65000,0,10000,0.2]


    def reset(self):
        self.rocket = Rocket() 
        return self.rocket.get_state()

    def step(self, action):
        """ 
        action0: accelerate, no turning, action1: decelerate, no turning
        action2: accelerate, turn left, action3: decelerate, turn left
        action4: accelerate, turn right, action5: decelerate, turn right
        return info -> int 
            0: flying 
            1: reached 
            2: boundary 
            3: crashed (height = 0 and x not in pad)
            4: unsafe landing 
        """
        reward = 0 
        info = 0 
        done = False
        if 0<= action <=5:
            x, y, theta = self.rocket.react(action)
        else:
            raise ValueError('ACTION WRONG')
        
        if self.check_reach() == True: 
            done = True 
            reward = self.REWARD_REACH
            info = 1 
            return [x,y,theta], reward, done, info
        
        if self.check_boundary() == True:
            done = True
            # print('Boundary hit')
            reward = self.REWARD_BOUND
            info = 2
            return [x,y,theta], reward, done, info

        if self.check_crash() == True:
            done = True
            # print('Crashed into Obstacle')
            reward = self.REWARD_CRASH
            info = 3
            return [x,y,theta], reward, done, info 

        if self.check_unsafe() == True:
            done = True 
            reward = self.REWARD_UNSAFE
            info = 4 
            return [x,y,theta], reward, done, info
        return [x,y,theta], reward, done, info

    def check_reach(self):
        """ check if the rocket lands safely """
        [xmin, xmax, ymin, ymax, delta] = self.SAFE_REGION
        if (xmin < self.rocket.x < xmax) and (ymin < self.rocket.y) < ymax and (-delta< self.rocket.theta < delta):
            return True 
        else:
            return False

    def check_boundary(self):
        """ Check if the rocket flies out of bound"""
        [xmax, ymax] = self.BOUNDS
        if self.rocket.x < 0 or self.rocket.x > xmax or self.rocket.y > ymax:
            return True 
        else:
            return False 
        
    def check_crash(self):
        """ check if the rocket crashed into the land or sea"""
        [xmin, xmax, ymin, ymax, delta] = self.SAFE_REGION
        if self.rocket.y <0:
            if not (xmin < self.rocket.x < xmax):
                return True 
        return False 

    def check_unsafe(self):
        [xmin, xmax, ymin, ymax, delta] = self.SAFE_REGION
        if (xmin < self.rocket.x < xmax) and (ymin < self.rocket.y < ymax):
            if not (-delta< self.rocket.theta < delta):
                return True 
        else: 
            return False

    def show_plot(self):
        xs = [i[0] for i in self.rocket.history]
        ys = [i[1] for i in self.rocket.history]
        plt.plot(xs,ys, 'b-')
        plt.plot(xs[0],ys[0],'r*')
        plt.xlim((0,self.BOUNDS[0]))
        plt.ylim((0,self.BOUNDS[1]))


        
