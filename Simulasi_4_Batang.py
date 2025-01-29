# Program Description:
# Simulate and animate the motion of four-bar linkage.
# The animation can also be stored as a video.

#--- Libraries 
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
from scipy.integrate import odeint # Bring in the differential equation solver


#--- Mechanism Parameters: [constants]

# Bar lengths
A_WIDTH = 1200
C_LENGTH = 1200
r_INPUT = 280
R_OUTPUT = 280

# Initial conditions [of bar r]  -Note: not all values are stable
r_ANG_INIT = 0.0
r_VEL_INIT = 0.10
r_TORQUE = 0.0
# Inertia of bars
INERTIA_r = 10
INERTIA_R = 1

#---- Simuation Parameters:

# Time interval
TIME_FINAL = 100
TIME_STEPS = 400

# Animation figures
BAR_WIDTH = 3
PIN_RADIUS = 30

#Plot bounds
X_BOUND = A_WIDTH
Y_BOUND = R_OUTPUT

# Save animation as .gif 
SAVE_FIGURE = True       # True or False

#--- Output angle function:

# Position parameter [ determined whether rotation is possible ]
def cos_input( r_ang ):
   #Variables for output calculation
   h_length = np.sqrt( A_WIDTH**2 + r_INPUT**2 
                      -2*A_WIDTH*r_INPUT*np.cos(r_ang) )    
     
   cos_term = ( ( R_OUTPUT**2 - C_LENGTH**2 + h_length**2 )/
               ( 2*R_OUTPUT*h_length ) )                     
   return cos_term


#   Global variable [angle at which bars lock]
g_lock_ang = 0
g_lock_switch = 0

#Function to calculate angle of bar R 
def output_angle( r_ang ):
 
   cos_term = cos_input(r_ang)

   if np.abs(cos_term) <= 1: 
       #Variables for output calculation   
       a_angle = np.arccos( cos_term )

       l_angle = np.arctan( np.sin(r_ang)*r_INPUT / 
                           ( A_WIDTH - r_INPUT*np.cos(r_ang) ) )
       #Output calculation:
       output = l_angle + a_angle
       
       #saving last non-locked angle
       global g_lock_ang
       g_lock_ang = r_ang

       return output
   else: 
       #Declaring bars are locked       
       global g_lock_switch
       g_lock_switch = 1
       
       return output_angle(g_lock_ang)
   #- End of function
    
#Function to limit angle of bar r if bar R is locked
def input_filter( r_ang ):
    if np.abs( cos_input(r_ang) ) > 1:
        return r_ang       #g_lock_ang
    else:
        return r_ang
    #- End of function


#--- Coordinates of pins:
origin = (0,0)
point_A = (A_WIDTH,0)

def point_r(r_ang):
    x = np.cos(r_ang)*r_INPUT
    y = np.sin(r_ang)*r_INPUT
    return (x, y)

def point_R(r_ang):
    pi = 3.14
    R_ang =pi - output_angle(r_ang)
    x = np.cos(R_ang)*R_OUTPUT + A_WIDTH
    y = np.sin(R_ang)*R_OUTPUT 
    return (x, y)


#--- Linkage arrays:
link_A = [ origin, point_A ]

def link_r(r_ang):
    return [ origin, point_r(r_ang) ]

def link_R(r_ang):
    return [ point_A, point_R(r_ang) ]

def link_C(r_ang):
    return [ point_r(r_ang), point_R(r_ang) ] 


#--- Motion functions

#Mechanical advantage between bar r and bar R    
def mech_ratio(r_ang):
    pi=3.14
    R_ang = pi - output_angle(r_ang)
    
    factor = r_INPUT*R_OUTPUT*np.cos(R_ang - r_ang)
    ratio = (A_WIDTH*r_INPUT*np.sin(r_ang) + factor)/(
            A_WIDTH*R_OUTPUT*np.sin(R_ang) + factor)
    return ratio

def engsel(r_ang):
    pi=3.14
    cos_term = cos_input(r_ang)
    a_angle = np.arccos( cos_term )
    l_angle = np.arctan( np.sin(r_ang)*r_INPUT / 
                           ( A_WIDTH - r_INPUT*np.cos(r_ang) ) )
    R_ang = pi-l_angle - a_angle
    
    return R_ang
   

#Derivatives of output_angle:
def deriv_1( r_ang ):
    dx = 1e-5
    return ( output_angle(r_ang+dx) - output_angle(r_ang-dx) )/(2*dx)    

def deriv_2(r_ang):
    dx= 1e-5
    return ( output_angle(r_ang+dx) + output_angle(r_ang-dx)
            - 2*output_angle(r_ang) )/(dx**2)


#--- Solution to bar motion

def stateVector_deriv( stateVector, t ):
    #Angle of bar r
    r_ang = stateVector[0];
    
    #checking if bars locked
    if g_lock_switch == 0:
        
        #Angular velocity of bar r
        r_ang_vel = stateVector[1];
    
        #Acceleration of bar r
        r_ang_accel = ( r_TORQUE - INERTIA_R*(r_ang_vel**2)*deriv_2(r_ang)*
                       mech_ratio(r_ang) )/(INERTIA_r + INERTIA_R*
                                 deriv_1(r_ang)*mech_ratio(r_ang) )
    
    #Setting motion to zero when bars locked
    else: 
        r_ang = g_lock_ang
        r_ang_vel = 0
        r_ang_accel = 0
    
    #U[0] = position ; U[1] = velocity
    #dU[0] = velocity ; dU[1] = acceleration
    return [ r_ang_vel, r_ang_accel ]

#Defining time interval
time_array = np.linspace(0, TIME_FINAL, TIME_STEPS)

#Solving state vector
stateVector_initial = [ r_ANG_INIT, r_VEL_INIT ]
stateVector_solution = odeint(stateVector_deriv, stateVector_initial, time_array)

#--- Creating figure for plot

# 1. animation
figure = plt.figure() 
axes = plt.axes( xlim=(-r_INPUT*1.1, X_BOUND + 1.2*R_OUTPUT), ylim=( -Y_BOUND - 1.2*R_OUTPUT, Y_BOUND + 1.2*R_OUTPUT ) )
plt.xlabel("X position")
plt.ylabel("Y position")
plt.title("Four bar linkage")

# 2. time plot
#figure1, axis1 = plt.subplots(nrows=1, ncols=1)
figure1, (axis1, axis2) = plt.subplots(2)
#figure1.suptitle('Vertically stacked subplots')
r_ang = stateVector_solution[:,0]
angvel = stateVector_solution[:,1]	# i=0 : angle | i = 1: angvel
Rr=engsel(r_ang)
axis1.plot( time_array,r_ang, time_array,Rr )
#axis1.set_xlabel("Time parameter")
axis1.set_ylabel("Crank Rotation (rad)")
axis1.set_title("Crank Rotation and Angular Velocity vs Time")

axis2.plot( time_array, angvel)
axis2.set_xlabel("Time parameter")
axis2.set_ylabel("Angular velocity of crank (rad/min)")


#--- Creating patch objects [ animation figures ]:
    
    #Linkages:
bar_A = plt.Polygon( link_A , closed = None, 
                    fill = None, ec = 'blue', lw = BAR_WIDTH )

bar_r = plt.Polygon( link_r(r_ANG_INIT), closed = None, 
                    fill = None, ec = 'red', lw = BAR_WIDTH )

bar_R = plt.Polygon( link_R(r_ANG_INIT), closed = None, 
                    fill = None, ec = 'gray', lw = BAR_WIDTH )

bar_C = plt.Polygon( link_C(r_ANG_INIT), closed = None,
                    fill = None, ec = 'purple', lw = BAR_WIDTH )

    #Pins:
pin_origin = plt.Circle( origin, PIN_RADIUS, fc = 'red' )
pin_A = plt.Circle( point_A, PIN_RADIUS, fc = 'red' )
pin_r = plt.Circle( point_r(r_ANG_INIT), PIN_RADIUS, fc = 'red' )
pin_R = plt.Circle( point_R(r_ANG_INIT), PIN_RADIUS, fc = 'red' )


#--- Animation functions:
def initial_plot():

    #Linkages:
    axes.add_patch( bar_A )
    axes.add_patch( bar_r )
    axes.add_patch( bar_R )

    #Pins:
    axes.add_patch( pin_origin )
    axes.add_patch( pin_A )

    #Layering bar C before rotating pins [r and R]
    axes.add_patch( bar_C )
    axes.add_patch( pin_r )
    axes.add_patch( pin_R )
    
    return bar_A, bar_r, pin_origin, bar_R, pin_A, bar_C, pin_R, pin_r
    #Note: return order is order of image layers
 
def animate_index(i):
    #output angle
    r_ang = stateVector_solution[i,0]
    #   Filtering input if bar R is locked
    #r_ang = input_filter(r_ang)

    #Linkages:
    bar_r.set_xy( link_r(r_ang) )
    bar_R.set_xy( link_R(r_ang) )
    bar_C.set_xy( link_C(r_ang) )

    #pins
    pin_r.center = point_r(r_ang) 
    pin_R.center = point_R(r_ang) 

    return bar_A, bar_r, pin_origin, bar_R, pin_A, bar_C, pin_R, pin_r
    #Note: return order is order of image layers


#--- Generating animation
anim = animation.FuncAnimation( figure, animate_index, init_func = initial_plot,
                                 frames = TIME_STEPS, interval = 40, blit = True )                             

#--- Saving animation [Remove comment to enable]
#if SAVE_FIGURE:
#	anim.save("figures/fourbar_anim.gif", writer=animation.PillowWriter(fps=5), dpi=75) 
#	figure1.savefig('figures/angvel_plot.jpg') 

# display figures
figure.show()
plt.show()

