# -*- coding: utf-8 -*-
"""
Way Point navigtion

(c) S. Bertrand
"""

import math
import Robot as rob
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import Timer as tmr
import Potential

# robot
x0 = -20.0
y0 = -20.0
#theta0 = np.pi/4.0
theta0 = np.pi
robot = rob.Robot(x0, y0, theta0)


# potential
pot = Potential.Potential(difficulty=1, random=False)


# position control loop: gain and timer
#kpPos = 0.8
kpPos = 1.7
positionCtrlPeriod = 0.2#0.01
timerPositionCtrl = tmr.Timer(positionCtrlPeriod)

# orientation control loop: gain and timer
#kpOrient = 2.5
kpOrient = 4.2
orientationCtrlPeriod = 0.05#0.01
timerOrientationCtrl = tmr.Timer(orientationCtrlPeriod)



# list of way points list of [x coord, y coord]
WPlist = [x0,y0]
#WPlist = []
CurPos=[robot.x,robot.y]# the current positon of the robot

#threshold for change to next WP
epsilonWP = 0.2
# init WPManager
WPManager = rob.WPManager([WPlist], epsilonWP)


# duration of scenario and time step for numerical integration
t0 = 0.0
tf = 500.0
dt = 0.01
simu = rob.RobotSimulation(robot, t0, tf, dt)


# initialize control inputs
Vr = 0.0
thetar = 0.0
omegar = 0.0

firstIter = True

robPosx = []
robPosy = []

theta1= np.pi/4.0
TarX = 0.0
TarY = 0.0
Pol = pot.value([robot.x, robot.y])
 
###############################CODE #############################################

def Target_point_m1(dist,angle) : ##point of view of the robot
    posX = robot.x
    posY = robot.y
    
    print(robot.x,robot.y)
    
    t_angle = robot.theta+angle
    print("\n t =",t_angle)
    
  
    
    print("\n a =",angle)
    
    print(robot.theta)
    print("\n t modif=",t_angle)
    #t_angle = t_angle - 2*np.pi
    cosi= dist*np.cos(t_angle)
    sini= dist*np.sin(t_angle)
    
    print("cosi",cosi)
    print("sini",sini)
    
    t_x = posX + cosi
    t_y = posY + sini
    
    return t_x,t_y



##verifiy if below 140 it works
def contour():
   
        if 140 <= Pol <= 160 :
            
            TarX,TarY=Target_point_m1(10.0,0)##go straight
            WPManager.WPList.append([TarX,TarY])
            
        elif Pol < 140 :
            TarX,TarY=Target_point_m1(10.0,theta1)
            WPManager.WPList.append([TarX,TarY])
        
        elif Pol>160 :
            TarX,TarY=Target_point_m1(10.0,theta1+np.pi)
            WPManager.WPList.append([TarX,TarY])






################################# END CODE ###########################################

# loop on simulation time
for t in simu.t: 
    robPosx.append(robot.x)
    robPosy.append(robot.y)
    Pol = pot.value([robot.x, robot.y]) 
    
    if WPManager.distanceToCurrentWP(robot.x, robot.y) < WPManager.epsilonWP :
       
       contour() 
        
       WPManager.switchToNextWP()
       
       
       
 
    
          
       #@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
       
       


    # position control loop
    if timerPositionCtrl.isEllapsed(t):

        potentialValue = pot.value([robot.x, robot.y])
        
        'ça sera mignon ça'
        
        # velocity control input
        Vr =kpPos*math.sqrt(math.pow( WPManager.xr - robot.x,2) +math.pow( WPManager.yr - robot.y,2))
        
        
        # reference orientation
        #thetar = theta0
        thetar=np.arctan2(WPManager.yr - robot.y,WPManager.xr - robot.x)
        
        
        if math.fabs(robot.theta-thetar)>math.pi:
            thetar = thetar + math.copysign(2*math.pi,robot.theta)        
        
        
        
    # orientation control loop
    if timerOrientationCtrl.isEllapsed(t):
        # angular velocity control input        
        omegar = kpOrient*(thetar - robot.theta)
    
    
    # assign control inputs to robot
    robot.setV(Vr)
    robot.setOmega(omegar)    
    
    # integrate motion
    robot.integrateMotion(dt)

    # store data to be plotted   
    simu.addData(robot, WPManager, Vr, thetar, omegar, pot.value([robot.x,robot.y]))
    
    
# end of loop on simulation time


# close all figures
plt.close("all")

# generate plots
fig,ax = simu.plotXY(1)
pot.plot(noFigure=None, fig=fig, ax=ax)  # plot potential for verification of solution

simu.plotXYTheta(2)
#simu.plotVOmega(3)

simu.plotPotential(4)



simu.plotPotential3D(5)


# show plots
#plt.show()





# # Animation *********************************

fig = plt.figure()
ax = fig.add_subplot(111, aspect='equal', autoscale_on=False, xlim=(-25, 25), ylim=(-25, 25))
ax.grid()
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')

robotBody, = ax.plot(robPosx ,robPosy , 'o-', lw=2)
robotDirection, = ax.plot([], [], '-', lw=1, color='k')
wayPoint, = ax.plot([], [], 'o-', lw=2, color='b')
time_template = 'time = %.1fs'
time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)
potential_template = 'potential = %.1f'
potential_text = ax.text(0.05, 0.1, '', transform=ax.transAxes)
WPArea, = ax.plot([], [], ':', lw=1, color='b')

thetaWPArea = np.arange(0.0,2.0*math.pi+2*math.pi/30.0, 2.0*math.pi/30.0)
xWPArea = WPManager.epsilonWP*np.cos(thetaWPArea)
yWPArea = WPManager.epsilonWP*np.sin(thetaWPArea)

def initAnimation():
    robotDirection.set_data([], [])
    robotBody.set_data([], [])
    wayPoint.set_data([], [])
    WPArea.set_data([], [])
    robotBody.set_color('r')
    robotBody.set_markersize(20)    
    time_text.set_text('')
    potential_text.set_text('')
    return robotBody,robotDirection, wayPoint, time_text, potential_text, WPArea  
    
def animate(i):  
    robotBody.set_data(simu.x[i], simu.y[i])          
    wayPoint.set_data(simu.xr[i], simu.yr[i])
    WPArea.set_data(simu.xr[i]+xWPArea.transpose(), simu.yr[i]+yWPArea.transpose())    
    thisx = [simu.x[i], simu.x[i] + 0.5*math.cos(simu.theta[i])]
    thisy = [simu.y[i], simu.y[i] + 0.5*math.sin(simu.theta[i])]
    robotDirection.set_data(thisx, thisy)
    time_text.set_text(time_template%(i*simu.dt))
    potential_text.set_text(potential_template%(pot.value([simu.x[i],simu.y[i]])))
    return robotBody,robotDirection, wayPoint, time_text, potential_text, WPArea

# ani = animation.FuncAnimation(fig, animate, np.arange(1, len(simu.t)),
#     interval=4, blit=True, init_func=initAnimation, repeat=False)
# #interval=25

# #ani.save('robot.mp4', fps=15)

