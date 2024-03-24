from modelversion2 import Robot, Ball, Shot

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider,Button

g = 9.81 #gravity [m/s^2]
M = 1 #mass [kg]
R = 1 #radius [m]

x0 = np.array([0,0,0]) #initial position [m]

N = 1000 #number of steps
t_bounds = (0,10) #time bounds [s]

k_hat = np.array([0,0,1]) #unit vector in z direction

###Plot
fig, ax = plt.subplots(figsize=(8,8))
fig.subplots_adjust(bottom=0.3,left=0.3) # adjust the main plot to make room for the sliders
ax.grid()
ax.set(xlabel="x [m]", ylabel="y [m]")



##Define slider axis
axcolor = 'lightgoldenrodyellow'
#horizontal axis
ax_v_kick = plt.axes([0.3, 0, 0.5, 0.03], facecolor=axcolor)
ax_theta_kick = plt.axes([0.3, 0.075, 0.5, 0.03], facecolor=axcolor)
ax_omega_drib = plt.axes([0.3, 0.15, 0.5, 0.03], facecolor=axcolor)

#vertical axis
ax_zoom = plt.axes([0.04, 0.31, 0.03, 0.5], facecolor=axcolor)
ax_mu = plt.axes([0.1, 0.31, 0.03, 0.5], facecolor=axcolor)

##Define slider parameters
slider_parameters = {'v_kick': {'initial': 10,'min': 0,'max': 100,'label':'v_kick','axis':ax_v_kick,'orientation':'horizontal','format': '%.1f m/s'}, 
                     'theta_kick': {'initial': np.pi/3,'min': 0,'max': np.pi,'label':'theta_kick','axis':ax_theta_kick,'orientation':'horizontal', 'format': '%.3f rad'},
                     'omega_drib': {'initial': 10,'min': 0,'max': 100,'label':'omega_drib','axis':ax_omega_drib,'orientation':'horizontal', 'format': '%.1f rad/s'	}, 
                     'zoom': {'initial': 10,'min': 1,'max': 100,'label':'zoom','axis':ax_zoom,'orientation':'vertical','format': '%.1f m'}, 
                     'mu': {'initial': 0.1,'min': 0,'max': 1,'label':'mu','axis':ax_mu,'orientation':'vertical','format': '%.3f'}
}

slider = {}
##Define sliders
for k,v in slider_parameters.items():
    slider[k] = Slider(v['axis'], v['label'], v['min'], v['max'], valinit=v['initial'],orientation=v['orientation'],valfmt=v['format'])

robot = Robot(x0,[0,0,0],[0,0,0],0)
ball = Ball(slider_parameters['mu']['initial'],M,R)
shot = Shot(robot,ball,np.array([slider_parameters['omega_drib']['initial'],0,0]),slider_parameters['v_kick']['initial'],slider_parameters['theta_kick']['initial'])
x, _, _, _ = shot.solve(t_bounds,N)


arrow = ax.arrow(0,0,10,10,head_width=2, head_length=2, fc='red', ec='red',label='initial velocity')
trajectory, = ax.plot(x[0],x[1],label='trajectory')


def update(val):
    global arrow
    v_kick = slider['v_kick'].val
    theta_kick = slider['theta_kick'].val
    omega_drib = np.array([slider['omega_drib'].val,0,0])
    mu = slider['mu'].val
    zoom = slider['zoom'].val

    ax.set(xlim=(-zoom,zoom),ylim=(0,zoom))
    ball = Ball(mu,M,R)
    shot = Shot(robot,ball,omega_drib,v_kick,theta_kick)
    x, _, _, _ = shot.solve(t_bounds,N)
    trajectory.set_data(x[0],x[1])

    arrow.remove()
    arrow = ax.arrow(0,0,0.5*zoom*np.cos(theta_kick),0.5*zoom*np.sin(theta_kick),
                     width=0.002*zoom ,head_width=0.06*zoom, head_length=0.06*zoom, 
                     fc='red', ec='red',label='initial velocity')

    fig.canvas.draw_idle()
    
update(0)

for k,v in slider.items():
    v.on_changed(update)

#Reset button
ax_reset = plt.axes([0, 0.96, 0.1, 0.04])
reset_button = Button(ax_reset, 'Reset', color='tab:red', hovercolor='0.975')

def reset(event):
    for k,v in slider.items():
        slider[k].reset()

reset_button.on_clicked(reset)


ax.legend()
plt.show()