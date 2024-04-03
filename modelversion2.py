import numpy as np
import matplotlib.pyplot as plt
import scipy as sp

k_hat = np.array([0,0,1]) #unit vector in z direction
g=9.81 # gravity [m/s^2]


class Robot():
    def __init__(self,x0,v0,omega0,ball_ofcenter): #ball_ofcenter is the distance from the center of the robot to the ball while driblling
        self.x0 = x0 #initial position [m]
        self.v0 = v0 #initial velocity [m/s]
        self.omega0 = omega0 #initial angular velocity [rad/s]
        self.ball_ofcenter = ball_ofcenter


class Ball():
    def __init__(self,mu,eta,M,R):
        self.mu = mu #friction coff []
        self.eta = eta #dyamic friction coff []
        self.M = M #mass [kg]
        self.R = R #radius [m]
        self.I = 2/5 * M * R**2 #moment of inertia [kg*m^2]

    def F_fric(self,v,omega):
        u = v - self.R * np.cross(omega, k_hat) #relative velocity to ground
        F = - (self.mu * self.M * g * u/np.sqrt(u.dot(u)) + self.eta * self.M * g * u) #friction force
        return F
    
    def T_rot(self,omega): #rotational energy
        T = omega @ self.I_tensor @ omega
        return T

    def T_cm(self,v): #Transversal of center of mass
        T = 1/2 * self.M * v @ v
        return T
    
    def T(self,v,omega): #total energy
        return self.T_rot(omega) + self.T_cm(v)
    
class Shot():
    def __init__(self,robot: Robot,ball: Ball,omega_drib: np.ndarray,v_kick: float,theta_kick: float):
        self.ball = ball #ball object
        self.robot = robot #robot object
        self.x0 = robot.x0 #initial position [m]
        self.v0 = v_kick * np.array([np.cos(theta_kick),np.sin(theta_kick),0]) + robot.v0 #!!!add robot omega!!!  #initial velocity [m/s]
        self.omega0 = omega_drib #initial angular velocity [rad/s]
        self.y0 = np.concatenate((self.x0,self.v0,self.omega0)) #initial state vector


    def _f(self,t,y):
        x = y[0:3] #position
        v = y[3:6] #velocity
        omega = y[6:9] #angular velocity

        F = self.ball.F_fric(v,omega) #friction force

        dx = v #derivative of position
        dv = F/self.ball.M #derivative of velocity
        domega = 1/self.ball.I * -self.ball.R * np.cross(k_hat, F) #derivative of angular velocity
        return np.concatenate((dx,dv,domega)) #return derivative of state vector
        
    def solve(self,t_bounds,N) -> 'tuple[np.ndarray,np.ndarray,np.ndarray]':
        t = np.linspace(*t_bounds,N) #time array
        sol = sp.integrate.solve_ivp(self._f,t_bounds,self.y0,t_eval=t) #solve ivp
        x = sol.y[0:3] #position
        v = sol.y[3:6] #velocity
        omega = sol.y[6:9] #angular velocity
        return x,v,omega,sol.t #return position, velocity, angular velocity, and time array