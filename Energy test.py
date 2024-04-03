from modelversion2 import Robot, Ball, Shot

import numpy as np

g = 9.81 #gravity [m/s^2]
M = 1 #mass [kg]
R = 1 #radius [m]

x0 = np.array([0,0,0]) #initial position [m]

N = 1000 #number of steps
t_bounds = (0,10) #time bounds [s]

k_hat = np.array([0,0,1]) #unit vector in z direction

mu = 0.1
v_kick = 10
theta_kick = np.pi/3
omega_drib = np.array([10,0,0])

robot = Robot(x0,[0,0,0],[0,0,0],0)
ball = Ball(mu,M,R)
shot = Shot(robot,ball,np.array(omega_drib),v_kick,theta_kick)
x, v, omega, _ = shot.solve(t_bounds,N)

print(x.shape)

print(ball.T(v[:,0],omega[:,0]))
print(ball.T(v[:,-1],omega[:,-1]))