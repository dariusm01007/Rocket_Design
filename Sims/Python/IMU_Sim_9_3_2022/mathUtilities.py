import numpy as np
import math

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
        
        # pitch == 90 -> Gimbal Lock
        # 1/cos(90) = undefined
        
        R = np.array([[1, np.sin(roll)*np.tan(pitch), np.cos(roll)*np.tan(pitch)], 
                      [0, np.cos(pitch), -np.sin(pitch)],
                      [0, np.sin(roll)/np.cos(pitch), np.cos(roll)/np.cos(pitch)]])
        
        # Body Rates [p,q,r].' from the gyroscope
        # The @ means 'matrix multiply'
        return R@BodyRate


