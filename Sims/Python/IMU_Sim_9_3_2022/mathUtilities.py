import numpy as np
import math
from numpy.linalg import matrix_power

def deg2rad(angle):
    return angle * (np.pi/180.0)

def rad2deg(angle):
    return angle* (180.0/np.pi)

def rotMat(angle, axis):
    if axis == 'x' or axis == 'X':
        # Roll 
        R = np.array([[1, 0, 0],
                     [0, np.cos(angle), np.sin(angle)], 
                     [0, -np.sin(angle), np.cos(angle)]])
        
    elif axis == 'y' or axis == 'Y':
        # Pitch 
        R = np.array([[np.cos(angle), 0, -np.sin(angle)], 
                     [0, 1, 0],
                     [np.sin(angle), 0, np.cos(angle)]])
        
    elif axis == 'z' or axis == 'Z':
        # Yaw
        R = np.array([[np.cos(angle), np.sin(angle),  0], 
                     [-np.sin(angle), np.cos(angle), 0],
                     [0, 0 ,1]])
        
    return R


def Inertial2Body(roll, pitch, yaw):
    # The @ means 'matrix multiply'
    return rotMat(roll,'x')@(rotMat(pitch, 'y')@rotMat(yaw, 'z'))
    
def Body2Inertial (roll, pitch, yaw):
    return np.transpose(Inertial2Body(roll, pitch, yaw))

def extractEuler(R):
    # Extracting Euler angles from Inertial2Body(roll, pitch, yaw)
    phi   = math.atan2(R[1,2],R[2,2])
    theta = math.asin(-R[0,2])
    psi   = math.atan2(R[0,1],R[0,0])
    
    return rad2deg(np.array([[phi],
                             [theta],
                             [psi]]))

def EulerKinematic(angles, BodyRate):
    
    roll  = angles[0].item()
    pitch = angles[1].item()
    
    # cos(pitch) == 90 -> Gimbal Lock
    # 1/cos(90) = undefined
    
    R = np.array([[1, np.sin(roll)*np.tan(pitch), np.cos(roll)*np.tan(pitch)], 
                  [0, np.cos(roll), -np.sin(roll)],
                  [0, np.sin(roll)/np.cos(pitch), np.cos(roll)/np.cos(pitch)]])
    
    # Body Rates [p,q,r].' from the gyroscope
    # The @ means 'matrix multiply'
    return R@BodyRate


def IMU_AttitudeUpdate(dcmOLD, Gyro, dt):
    
    # Skew-symmetric matrix of the angular rate vector
    wx = Gyro[0].item()
    wy = Gyro[1].item()
    wz = Gyro[2].item()
    
    OMEGA = np.array([[0, -wz,  wy], 
                      [wz, 0,  -wx],
                      [-wy, wx ,0]])
    
    # Identity Matrix    
    I = np.identity(3)
        
    incr = I + OMEGA*dt + (matrix_power(OMEGA*dt, 2)*(1/np.math.factorial(2)))
    
    # Remember dcmOLD & dcmNEW are in the body frame (gyro measures body rates)
    # May need to do : Body2Inertial(roll, pitch, yaw) first 
    
    dcmNEW = dcmOLD@incr
    
    # Extracting Euler angles from Body2Inertial(roll, pitch, yaw)
    phi   = math.atan2(dcmNEW[2,1],dcmNEW[2,2])
    theta = math.asin(-dcmNEW[2,0])
    psi   = math.atan2(dcmNEW[1,0],dcmNEW[0,0])
    
    return rad2deg(np.array([[phi],
                             [theta],
                             [psi]]))