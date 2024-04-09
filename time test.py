from modelversion2 import Robot, Ball, Shot

from time import time
import numpy as np

g = 9.81 #gravity [m/s^2]
M = 0.0459#mass [kg]
R = 0.0427/2 #radius [m]

x0 = np.array([0,0,0]) #initial position [m]

N = 1000 #number of steps
t_bounds = (0,10) #time bounds [s]

k_hat = np.array([0,0,1]) #unit vector in z direction

slider_parameters = {'v_kick': {'initial': 10,'min': 0,'max': 100,'label':'v_kick'}, 
                     'theta_kick': {'initial': np.pi/3,'min': 0,'max': np.pi,'label':'theta_kick'},
                     'omega_drib': {'initial': 4000,'min': 2000,'max': 6000,'label':'omega_drib'}, 
                     'zoom': {'initial': 10,'min': 1,'max': 100,'label':'zoom'}, 
                     'mu': {'initial': 0.1,'min': 0,'max': 1,'label':'mu'},
                     'eta': {'initial': 0.1,'min': 0,'max': 1,'label':'eta'},
}

robot = Robot(x0,[0,0,0],[0,0,0],0)
ball = Ball(slider_parameters['mu']['initial'],slider_parameters['eta']['initial'],M,R)
shot = Shot(robot,ball,np.array([slider_parameters['omega_drib']['initial'],0,0]),slider_parameters['v_kick']['initial'],slider_parameters['theta_kick']['initial'])

t1 = time()
x, _, _, _,sol = shot.solve(t_bounds,N)
t2 = time()

print(t2-t1)