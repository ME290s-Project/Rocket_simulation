'''
 # @ Author: Zion Deng
 # @ Description: simulation using matplotlib
 '''

from matplotlib import pyplot as plt
import numpy as np 
from matplotlib import patches as mpatches
from cmath import pi, sin, cos

DT = 0.01  # the fresh time 

class RocketPlt():
    """ Rocket class  """
    def __init__(self):
        self.SIZE = np.array([0.4,1])
        self.BODY_LENGTH = 0.4 
        self.WING_LENGTH = 0.2
        self.position = np.array([2,2.2])
        self.body_angle = 0  # the angle of the body 
        self.wing_angle = 0 # the angle of the wing
        self.main_engine_on = False 
        self.full_engine_on = False

    def render_rocket(self):
        bottom_point = self.position
        top_point_x = self.position[0] + self.BODY_LENGTH * sin(self.body_angle)
        top_point_y = self.position[1] + self.BODY_LENGTH * cos(self.body_angle)
        body_line_x = [bottom_point[0], top_point_x]
        body_line_y = [bottom_point[1], top_point_y]
        plt.plot(body_line_x, body_line_y, c = 'blue', linewidth = 10) # draw the body 

        # draw wings 
        wing_angle = self.wing_angle + self.body_angle 
        wing_point_x = top_point_x - self.WING_LENGTH *sin(wing_angle)
        wing_point_y = top_point_y - self.WING_LENGTH *cos(wing_angle)
        wing_line_x = [top_point_x,wing_point_x]
        wing_line_y = [top_point_y,wing_point_y]
        plt.plot(wing_line_x, wing_line_y, c = 'green',linewidth = 2)

        # draw engine fires 
        if self.full_engine_on:
            fire_length = 0.3 
            fire_width = 8
        elif self.main_engine_on:
            fire_length = 0.2
            fire_width = 5
        else: 
            fire_length = 0 
            fire_width = 0 

        fire_point_x = self.position[0] - fire_length * sin(self.body_angle)
        fire_point_y = self.position[1] - fire_length * cos(self.body_angle)
        fire_line_x = [self.position[0],fire_point_x]
        fire_line_y = [self.position[1],fire_point_y]
        plt.plot(fire_line_x, fire_line_y, c = 'red', linewidth = fire_width)
        

def canvas_init():
    """ draw obstacles and target """
    plt.cla()
    plt.axis('equal')
    plt.axis('off')
    plt.title('ROCKET SIMULATION')
    plt.plot([0,0,10,10,0], [0,10,10,0,0], color = 'black')  # draw boundary
    # draw land
    land = mpatches.Rectangle(land_point, 4,2, color = 'y')
    ax.add_patch(land)
    # draw sea
    sea = mpatches.Rectangle(sea_point, 6,2, color = 'b')
    ax.add_patch(sea)
    # draw land pad 
    land_pad = mpatches.Rectangle(land_pad_point,2,0.2,color = 'g')
    ax.add_patch(land_pad)
    # draw sea pad
    sea_pad = mpatches.Rectangle(sea_pad_point,2,0.2,color = 'g')
    ax.add_patch(sea_pad)

    



plt.ion()
fig, ax = plt.subplots() 
land_point = np.array([0.0, 0.0])  # the bottom left position of land
sea_point = np.array([4.0, 0.0])   # the bottom left position of sea
land_pad_point = np.array([2,2])
sea_pad_point = np.array([7,2])
rocket = RocketPlt()

def simulation2D():
    while True:
        canvas_init()
        # rocket.wing_angle += 0.1
        # rocket.body_angle -= 0.1
        # rocket.main_engine_on = True
        rocket.render_rocket()
        plt.pause(DT)
        # plt.savefig('pic.jpg')

if __name__ == '__main__':
    simulation2D()