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
dt = 2**(-10) # floating point error
sprung_v = [] # empty lists to be used for appending and then plotting (lists are mutable so useful here)
sprung_z = []
unsprung_v = []
unsprung_z = []
time_array = np.arange(0, 5, dt)

#----- Time response Actual Calculator, Velocity-Verlet integration for accuracy ------#

for t in time_array:

    
    z_r = Amp * np.sin(omega * t) # compute z_r and v_r at time t, modelling as sine wave for now
    v_r = Amp * omega * np.cos(omega * t)
    
    a_s = (1/m_s) * (k_s * (z_u - z_s) + b_s * (v_u - v_s))
    a_u = (1/m_u) * (k_s * (z_s - z_u) + b_s * (v_s - v_u) + k_u * (z_r - z_u) + b_u * (v_r - v_u))
    
    z_s += v_s * dt + 0.5 * a_s * dt ** 2
    z_u += v_u * dt + 0.5 * a_u * dt ** 2
    
    # --- Predict --- #
    v_s_pred = v_s + a_s * dt # FIXED: New variable defined such that v_s is still from last loop not overwritten within the loop before computed
    v_u_pred = v_u + a_u * dt
    
    t_next = t + dt
    z_r_new = Amp * np.sin(omega * t_next)
    v_r_new = Amp * omega * np.cos(omega * t_next)
    
    a_s_new = (1/m_s) * (k_s * (z_u - z_s) + b_s * (v_u_pred - v_s_pred)) # Using predicted v(t+dt) to work out a pseudo a(t+dt) just for use here, calculated properly at next loop
    a_u_new = (1/m_u) * (k_s * (z_s - z_u) + b_s * (v_s_pred - v_u_pred) + k_u * (z_r_new - z_u) + b_u * (v_r_new - v_u_pred))
    
    # --- Correct --- #
    v_s = v_s + 0.5 * (a_s + a_s_new) * dt
    v_u = v_u + 0.5 * (a_u + a_u_new) * dt
    # Perform the correction using the correct Verlet algorithm with our predicted acceleration, now the correct velocities are ready to be appended

    sprung_v.append(v_s) # append to the empty lists
    sprung_z.append(z_s)
    unsprung_v.append(v_u)
    unsprung_z.append(z_u)
    
# --- Convert to NumPy arrays for faster post processing --- #

array_sprung_v = np.array(sprung_v)
array_unsprung_v = np.array(unsprung_v)
array_unsprung_z = np.array(unsprung_z)
array_sprung_z = np.array(sprung_z)

# ----- Plotting ----- #
#
plt.figure(1)
plt.plot(time_array,array_sprung_v, time_array, array_unsprung_v)
plt.title ('Velocities')
plt.xlabel('Time')
plt.ylabel('Velocity')
plt.legend()

plt.figure(2)
plt.plot(time_array, array_sprung_z, time_array, array_unsprung_z)
plt.title ('Positions')
plt.xlabel('Time')
plt.ylabel('Position')
plt.legend()

plt.show()




    
    


    