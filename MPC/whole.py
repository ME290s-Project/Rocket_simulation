'''
 # @ Author: Zion Deng
 # @ Description: Whole process simulation
 '''

from MPC_FT import MPC_FT
from MPC_glide import MPC_glide 
from MPC_reen import MPC_reen
from MPC_land import MPC_land
import numpy as np 
import matplotlib.pyplot as plt 

def get_position(name,xs, ys, c):
    feas, xOpt, uOpt = eval('MPC_'+name)()
    x_pos = xOpt[0]
    y_pos = xOpt[1]
    xs = np.hstack((xs,x_pos))
    ys = np.hstack((ys,y_pos))
    tangentdx = xs[-3] - xs[-1]
    tangentdy = ys[-3] - ys[-1]
    tangentx = [xs[-1] - 3 * tangentdx, xs[-1] + 10 * tangentdx]
    tangenty = [ys[-1] - 3 * tangentdy, ys[-1] + 10 * tangentdy]
    plt.plot(x_pos, y_pos, color = c, label = name)
    plt.plot(tangentx, tangenty, 'm--')
    return xs, ys


def simulation():
    """ rocket launch simulation
    return: position and input """
    xs = np.array([0])
    ys = np.array([200])
    fnames = ['FT','glide','reen','land']
    colors = ['r','g','b','y']


    for name,color in zip(fnames,colors):
        xs, ys = get_position(name,xs, ys,color)
    
    # plt.plot(xs,ys)
    plt.plot(0,200,'r*',label = 'start')
    plt.plot(xs[-1],ys[-1],'yo',label = 'end')
    plt.legend()
    plt.title('Trajectory for Whole Process')
    plt.show() 


if __name__ == '__main__':
    simulation() 


