import numpy as np
import matplotlib.pyplot as plt
from mathUtilities import deg2rad, EulerKinematic, rad2deg, Inertial2Body

# Initial Orientation
roll  = deg2rad(10) # [deg]
pitch = deg2rad(35) # [deg]
yaw   = deg2rad(87) # [deg]

# Time Related Parameters
dt     = 0.01  # [s]
t      = 0.0   # [s]
tFinal = 120.0 # [s]

# Determining size for memory pre-allocation (for speed)
arrayLength = int((tFinal - t)/dt)

# Pre-Allocating memory
phi      = np.zeros(arrayLength)
theta    = np.zeros(arrayLength)
psi      = np.zeros(arrayLength)
phiDot   = np.zeros(arrayLength)
thetaDot = np.zeros(arrayLength)
psiDot   = np.zeros(arrayLength)

p_array  = np.zeros(arrayLength)
q_array  = np.zeros(arrayLength)
r_array  = np.zeros(arrayLength)

time     = np.zeros(arrayLength)

g_vector = np.zeros((arrayLength,3))

angles = np.array([[roll],
                   [pitch],
                   [yaw]])

# Down is positive
gravity = np.array([[0.0],
                    [0.0],
                    [9.81]])

# Quick Lambda function to convert [m/s^2] to G's
accel2G = lambda a : a/9.81

 
for idx in range (0,arrayLength):
    
    # Simulating Gyroscope (no noise, bias, drift, etc...)
    if t <= 5.0:
        
        # Not rolling
        p = 0.0 
        
        # Not pitching
        q = 0.0 # 
        
        # Turning to the right
        r = deg2rad(5) # 
        
    elif t > 5.0 and t <= 15.0:
        
        # Rolling counterclockwise
        p = -deg2rad(0.8)
        
        # Not pitching
        q = 0.0
        
        # Turning to the left
        r = -deg2rad(3.5)
        
    elif t > 15 and t <= 100:
        
        # Rolling clockwise
        p = deg2rad(0.5)
        
        # Pitching Downward
        q = -deg2rad(0.002)
        
        # Stop turning for a while
        r = 0.0
        
    else:
        
        # Not rolling
        p = 0.0
        
        # Pitching upward
        q = deg2rad(0.025)
        
        # Turning to the right
        r = deg2rad(0.25)
        
    BodyRates = np.array([[p],
                          [q],
                          [r]])
    
    # Storing body rates
    p_array[idx] = rad2deg(p)
    q_array[idx] = rad2deg(q)
    r_array[idx] = rad2deg(r)
    
    # Extracting attitude rates
    AttitudeRates = EulerKinematic(angles, BodyRates)
    
    # Integrating attitude rates to update Euler angles
    angles += AttitudeRates*dt
    
    # Incrementing time
    t += dt
    
    # Storing values
    phi[idx]      = rad2deg(angles[0].item())
    theta[idx]    = rad2deg(angles[1].item())
    psi[idx]      = rad2deg(angles[2].item())
    phiDot[idx]   = rad2deg(AttitudeRates[0].item())
    thetaDot[idx] = rad2deg(AttitudeRates[1].item())
    psiDot[idx]   = rad2deg(AttitudeRates[2].item())
    time[idx]     = t
    
    # Simulating accelerometer (no external acceleration, noise, etc...)
    # Storing gravity components in the body frame
    g_vector[idx, :] = np.transpose((Inertial2Body(phi[idx], theta[idx], psi[idx])@gravity))

# Plotting values
plt.figure(1)
plt.plot(time, phi)
plt.ylabel('Angle [deg]')
plt.xlabel('Time [s]')
plt.title('Roll Angle')
plt.grid()
plt.show()

plt.figure(2)
plt.plot(time, theta)
plt.ylabel('Angle [deg]')
plt.xlabel('Time [s]')
plt.title('Pitch Angle')
plt.grid()
plt.show()

plt.figure(3)
plt.plot(time, psi)
plt.ylabel('Angle [deg]')
plt.xlabel('Time [s]')
plt.title('Yaw Angle')
plt.grid()
plt.show()

# Comparing Body Rates to Attitude Rates
plt.figure(4)
plt.plot(time, phiDot, label = 'Attitude Rate')
plt.plot(time, p_array,'r--', label = 'Body Rate')
plt.ylabel('Angular Rate [deg/s]')
plt.xlabel('Time [s]')
plt.title('Roll Rate')
plt.legend()
plt.grid()
plt.show()

plt.figure(5)
plt.plot(time, thetaDot, label = 'Attitude Rate')
plt.plot(time, q_array,'r--', label = 'Body Rate')
plt.ylabel('Angular Rate [deg/s]')
plt.xlabel('Time [s]')
plt.title('Pitch Rate')
plt.legend()
plt.grid()
plt.show()

plt.figure(6)
plt.plot(time, psiDot, label = 'Attitude Rate')
plt.plot(time, r_array,'r--', label = 'Body Rate')
plt.ylabel('Angular Rate [deg/s]')
plt.xlabel('Time [s]')
plt.title('Yaw Rate')
plt.legend()
plt.grid()
plt.show()

# Plotting gravity components
plt.figure(7)
plt.plot(time, accel2G(g_vector[:,0]))
plt.ylabel('Acceleration [G]')
plt.xlabel('Time [s]')
plt.title('X-Axis Acceleration due to gravity')
plt.grid()
plt.show()

plt.figure(8)
plt.plot(time, accel2G(g_vector[:,1]))
plt.ylabel('Acceleration [G]')
plt.xlabel('Time [s]')
plt.title('Y-Axis Acceleration due to gravity')
plt.grid()
plt.show()

plt.figure(9)
plt.plot(time, accel2G(g_vector[:,2]))
plt.ylabel('Acceleration [G]')
plt.xlabel('Time [s]')
plt.title('Z-Axis Acceleration due to gravity')
plt.grid()
plt.show()