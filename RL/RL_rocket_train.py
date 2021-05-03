'''
 # @ Author: Zion Deng
 # @ Description: RL training for Rocket landing using DQN 
 '''

# import os,sys 
# sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
#                 "/Path_Utils")
# from math import pi
import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np  
import matplotlib.pyplot as plt 
from RL_rocket_env import RocketEnv
from numpy import average
# from Testing import realtime_search
import time 

# Hyper Parameters
N_EPISODES = 300000            # Number of total episodes
N_SHOWN = 50  # show results for N steps

BATCH_SIZE = 32
LR = 0.01                   # learning rate
EPSILON = 0.85              # greedy policy
GAMMA = 0.99               # reward discount, the larger, the longer sight. 
TARGET_REPLACE_ITER = 50   # target update frequency
MEMORY_CAPACITY = 100

N_ACTIONS = 6
N_STATES = 3
ENV_A_SHAPE = 0
DESIRED_ACCURACY = 0.3   # the desired accuracy of training results


class Net(nn.Module):
    def __init__(self):
        super().__init__()
        self.fc1 = nn.Linear(N_STATES, 30)
        self.fc1.weight.data.normal_(0, 0.1)  # initialization
        self.fc2 = nn.Linear(30, 30)
        self.fc2.weight.data.normal_(0,0.1)
        self.out = nn.Linear(30, N_ACTIONS) 
        self.out.weight.data.normal_(0,0.1) 

    def forward(self, x):
        x = self.fc1(x) 
        x = F.relu(x)  # activation 
        x = self.fc2(x) 
        x = F.relu(x)  # activation 
        actions_value = self.out(x) 
        return actions_value


class DQN():
    def __init__(self):
        self.eval_net, self.target_net = Net(), Net()

        self.learn_step_counter = 0 
        self.memory_counter = 0 
        self.memory = np.zeros((MEMORY_CAPACITY,N_STATES * 2+2))
        self.optimizer = torch.optim.Adam(self.eval_net.parameters(), lr = LR)
        self.loss_func = nn.MSELoss()

    def choose_action(self,x): 
        x = torch.unsqueeze(torch.FloatTensor(x),0) 
        # input only one sample 
        if np.random.uniform() < EPSILON:  # greedy 
            actions_value = self.eval_net.forward(x) 
            action = torch.max(actions_value, 1)[1].data.numpy()
            action = action[0] if ENV_A_SHAPE == 0 else action.reshape(ENV_A_SHAPE)  # return the argmax index        
        else: # random 
            action = np.random.randint(0,N_ACTIONS) 
            action = action if ENV_A_SHAPE == 0 else action.reshape(ENV_A_SHAPE) 

        return action 

    def store_transition(self, s,a,r,s_):
        transition = np.hstack((s,[a,r], s_))
        # replace the old memory with new memory
        index = self.memory_counter % MEMORY_CAPACITY 
        self.memory[index, :] = transition 
        self.memory_counter += 1 

    def learn(self):
        if self.learn_step_counter % TARGET_REPLACE_ITER == 0:
            self.target_net.load_state_dict(self.eval_net.state_dict())
        self.learn_step_counter += 1 

        # sample batch transitions 
        sample_index = np.random.choice(MEMORY_CAPACITY, BATCH_SIZE) 
        b_memory = self.memory[sample_index,:] 
        b_s = torch.FloatTensor(b_memory[:,:N_STATES])
        b_a = torch.LongTensor(b_memory[:,N_STATES:N_STATES+1].astype(int))
        b_r = torch.FloatTensor(b_memory[:,N_STATES+1:N_STATES +2])
        b_s_ = torch.FloatTensor(b_memory[:, -N_STATES:])

        # q_eval w.r.t the action in experience 
        q_eval = self.eval_net(b_s).gather(1,b_a)  # shape:(batch, 1)
        q_next = self.target_net(b_s_).detach()    # detach from graph, don't backpropagate
        q_target = b_r + GAMMA * q_next.max(1)[0].view(BATCH_SIZE,1)   # shape (batch, 1)
        loss = self.loss_func(q_eval,q_target) 

        self.optimizer.zero_grad() 
        loss.backward() 
        self.optimizer.step()

def train():
    """ RL training with DQN """
    env = RocketEnv() 
    dqn = DQN()
    events = np.array([0,0,0,0]) # record the events <- 
    events_history = events.copy()
    accuracy_history = [0.0]
    for i in range(N_EPISODES):
        s = env.reset() 
        ep_r = 0  # episode reward 
        step = 0  # step count
        while True: 
            step += 1 
            a = dqn.choose_action(s) 
            s_,r,done,info = env.step(a) 
            dqn.store_transition(s,a,r,s_)
            if dqn.memory_counter > MEMORY_CAPACITY:
                dqn.learn() 
            
            ep_r += r 
            if done: # info 1 means reached the goal 
                print('\rTraining Episode now: %d, Event Number is: %d' %(i,info), end = '')
                events[info-1] += 1 
                # plt.cla() 
                # env.show_plot() 
                # plt.pause(0.001)
                break 

        if i % N_SHOWN == 0:  # show the results in N_shown episode
            episode_events = events - events_history
            events_history = events.copy()
            accuracy = episode_events[0] / sum(episode_events)
            print('\n Results: Reached: {0} Boundary: {1}, Crashed: {2}, Unsafe: {3}'.format(
                episode_events[0], episode_events[1], episode_events[2], episode_events[3]))
            print('ACCURACY: {0}\n'.format(accuracy))
            accuracy_history.append(accuracy)


if __name__ == '__main__':
    train() 