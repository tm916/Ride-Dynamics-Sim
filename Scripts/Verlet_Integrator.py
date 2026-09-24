# NO DOWNFORCE, yet... #
import numpy as np
import matplotlib.pyplot as plt

def simulate (k_s, b_s, end_time=5, dt = 2**(-10), omega = 31.4): # Pass in the damper and spring values
    
    # --- Setting some values I don't expect to change, loosely based off of formula student --- # 
    Amp = 0.005 # Amplitude of some bumps
    m_u = 11.2 # Estimate
    m_s = 68.8 # Driver + car approx 320kg for Cambridge, divide by 4, subtract 11.2
    k_u = 100000
    b_u = 150 # damper uses b instead of "lambda" for obvious reasons
    
    # ---- Initial conditions ----- #
    v_u = v_r = v_s = a_s = a_u = z_u = z_r = z_s = 0
    
    sprung_v, unsprung_v, sprung_z, unsprung_z = [], [], [], [] # Empty lists to begin with
    time_array = np.arange(0, end_time, dt)
    
    for t in time_array:

        sprung_v.append(v_s) # Sync time
        sprung_z.append(z_s)
        unsprung_v.append(v_u)
        unsprung_z.append(z_u)
        
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

        
    # --- Convert to NumPy arrays for faster post processing --- #

    array_sprung_v = np.array(sprung_v)
    array_unsprung_v = np.array(unsprung_v)
    array_unsprung_z = np.array(unsprung_z)
    array_sprung_z = np.array(sprung_z)

    #----- Find steady state amplitude using last 5 data points ---#

    T = 2 * np.pi / omega    # road period to be used below to find amplitude from last 5 cycles
    mask = time_array >= time_array[-1] - 5 * T # Boolean indexing
    z_ss = array_sprung_z[mask] 
    amp_body = 0.5 * (z_ss.max() - z_ss.min()) # Amp, sinusoid, half range
    
    return time_array, array_sprung_z, array_unsprung_z, array_sprung_v, array_unsprung_v, amp_body # All needed for analysis

    
    
    
# ----- Plotting ----- #
time_array, array_sprung_z, array_unsprung_z, array_sprung_v, array_unsprung_v, amp_body = simulate(25000, 1500) #default run for later plot

plt.figure(1)
plt.plot(time_array, array_sprung_v, label='Sprung mass')
plt.plot(time_array, array_unsprung_v, label='Unsprung mass')
plt.title ('Velocities')
plt.xlabel('Time(s)')
plt.ylabel('Velocity(m/s)')
plt.legend()

plt.figure(2)
plt.plot(time_array, array_sprung_z, label='Sprung mass')
plt.plot(time_array, array_unsprung_z, label='Unsprung mass')
plt.title ('Displacement From Equilibrium')
plt.xlabel('Time(s)')
plt.ylabel('Displacement(m)')
plt.legend()

plt.show()

# ----- Parameter study: spring rate and damping at fixed frequency ----- #

# NOTE: k_s and b_s are WHEEL rates, not spring/damper rates.
# Spring sweep: ride rate = k_s*k_u/(k_s + k_u) with tyre in series
# Gives ride frequency ~2.2-3.2 Hz for m_s = 68.8 kg
# Plausible for an FS car without much aero
k_values = np.linspace(15000, 40000, 6)   # N/m (wheel rate)

# Damper sweep: zeta = b_s / (2*sqrt(k_ride*m_s))
# Gives zeta ~0.2-1.3 at baseline spring, typical ride target ~0.5-0.8
b_values = np.linspace(500, 3000, 6)      # Ns/m (wheel rate)

results = []

for k in k_values:
    for b in b_values:
        *_, amp = simulate(k, b)          # *_ discards all outputs except the last
        results.append([k, b, amp])

results = np.array(results)
np.savetxt('amplitude_study_final.csv', results, delimiter=',',
           header='k_s,b_s,amp_body', comments='')

# ----- More plotting -----
plt.figure(3)
for k in k_values:
    rows = results[:, 0] == k
    plt.plot(results[rows, 1], results[rows, 2] / 0.005, marker='o',
             label=f'k_s = {k:.0f} N/m')
plt.xlabel('Damper wheel rate b_s (Ns/m)')
plt.ylabel('Body amplitude / road amplitude')
plt.title('Body amplitude ratio, 5 Hz road input')
plt.legend()
plt.show()
# Extra for the comparision with Simulink model
np.savetxt('python_zs.csv', np.column_stack([time_array, array_sprung_z]),
           delimiter=',', header='t,z_s', comments='')
    


    