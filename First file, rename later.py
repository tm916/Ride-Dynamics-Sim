# -- initial simulation -- NO DOWNFORCE #
import numpy as np
import matplotlib.pyplot as plt
Amp = 0.02 # amplitude
omega = 31.4
m_u = 11.2
m_s = 95.2
k_s = 25000 # spring stiffnesses and damping constants
b_s = 1500
k_u = 100000
b_u = 150 # damper uses b instead of "lambda" for obvious reasons
v_u = 0 #initialising initial positions etc.
v_s = 0
v_r = 0
a_u = 0
a_s = 0
z_u = 0
z_r = 0
z_s = 0
dt = 2**(-10) #cos floating point error innit
sprung_v = [] # empty lists to be used for appending and then plotting (lists are mutable so useful here)
sprung_z = []
unsprung_v = []
unsprung_z = []
time_array = np.arange(0, 5, dt)

#----- Actual Calculator ------#

for i in range (len(time_array)):

    t = time_array[i] # hopefully this iterates through the time array 
    z_r = Amp * np.sin(omega * t) # compute z_r and v_r at time t, modelling as sine wave for now
    v_r = Amp * omega * np.cos(omega * t)
    
    a_s = (1/m_s) * (k_s * (z_u - z_s) + b_s * (v_u - v_s))
    a_u = (1/m_u) * (k_s * (z_s - z_u) + b_s * (v_s - v_u) + k_u * (z_r - z_u) + b_u * (v_r - v_u))

    v_s += a_s * dt # euler-cromer method for numerical integration
    v_u += a_u * dt

    z_s += v_s * dt # ...and for position
    z_u += v_u * dt
    
    sprung_v.append(v_s) # append to the empty lists
    sprung_z.append(z_s)
    unsprung_v.append(v_u)
    unsprung_z.append(z_u)
    
# ----- plotting ----- #
plt.figure(1)
plt.plot(time_array,sprung_v, time_array, unsprung_v)
plt.title ('Velocities')
plt.xlabel('Time')
plt.ylabel('Velocity')
plt.legend()

plt.figure(2)
plt.plot(time_array, sprung_z, time_array, unsprung_z)
plt.title ('Positions')
plt.xlabel('Time')
plt.ylabel('Position')
plt.legend()

plt.show()
    