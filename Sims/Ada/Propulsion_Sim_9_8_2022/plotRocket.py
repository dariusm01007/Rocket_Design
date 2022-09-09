import pandas as pd
import matplotlib.pyplot as plt

def stateData(fileName):
    # importing the state data
    dataframe = pd.read_csv(fileName, delimiter= '\s+')
    
    # converting the info from a pandas dataframe into a numpy array
    stateArray = dataframe.to_numpy()
    
    return stateArray

stateArray = stateData("rocketPropulsionSim.txt")

time         = stateArray[:,0]
altitude     = stateArray[:,1]
downrange    = stateArray[:,2]
flightPath   = stateArray[:,3]
velocity     = stateArray[:,4]
acceleration = stateArray[:,5]
mass         = stateArray[:,6]

plt.figure(1)
plt.plot(downrange, altitude)
plt.ylabel("Altitude [m]")
plt.xlabel("Downrange [m]")
plt.title("Rocket Trajectory")
plt.grid()
plt.show()

plt.figure(2)
plt.plot(time, flightPath)
plt.ylabel("Angle [deg]")
plt.xlabel("Time [s]")
plt.title("Flight Path Angle")
plt.grid()
plt.show()

plt.figure(3)
plt.plot(time, velocity)
plt.ylabel("Velocity [m/s]")
plt.xlabel("Time [s]")
plt.title("Rocket Velocity")
plt.grid()
plt.show()

plt.figure(4)
plt.plot(time, acceleration)
plt.ylabel("Acceleration [G]")
plt.xlabel("Time [s]")
plt.title("Rocket Acceleration")
plt.grid()
plt.show()

plt.figure(5)
plt.plot(time, mass)
plt.ylabel("Mass [kg]")
plt.xlabel("Time [s]")
plt.title("Rocket Mass")
plt.grid()
plt.show()