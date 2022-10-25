from multiprocessing.connection import wait
from tkinter import W
import numpy as np
from numpy import sin, cos, tan, pi, arccos, arcsin, sqrt, radians, degrees
from scipy.linalg import expm, logm
import matplotlib.pyplot as plt
from fractions import Fraction

np.set_printoptions(precision=3, suppress=True)

def skew(w):
    '''
    Get skew symmetric [ω̂] (3x3 matrix) of a rotation axis w (3 vector). \n
    exp([ω̂]θ) is a rotation matrix of θ radians around axis ω. \n
    ṗ = [ω̂]p \n
    p(θ) = exp([ω̂]θ)*p(0)
    '''
    w = np.array(w).flatten()
    skv = np.roll(np.roll(np.diag(w), 1, 1), -1, 0)
    return skv - skv.T

def unskew(W):
    '''
    Get axis w (3 vector) by extracting components out of a skew symmetric [ω̂] (3x3 matrix)
    '''
    w = np.empty(3)
    w[0] = W[2][1]
    w[1] = W[0][2]
    w[2] = W[1][0]
    return w

def get_rotation_from_axis_angle(axis, angle):
    '''
    Rodriques formula for rotations. Get the Rotation (3x3 matrix) by evaluating exp([ω̂]θ).
    axis: [ω̂] (3x3 skew matrix)
    angle: θ  (scalar angle)
    '''
    W = axis
    theta = angle
    R = np.eye(3) + sin(theta)*W + (1 - cos(theta))*(W@W)
    return R

def get_axis_angle_from_rotation(R):
    '''
    Get [ω̂] (3x3 skew matrix) and θ (scalar) from a rotation (3x3 matrix) by taking matrix log.
    '''
    theta = arccos((np.trace(R) - 1) / 2)
    W = (R - R.T)/ (2 * sin(theta))
    axis = W
    angle = theta
    return axis, angle



w = [0, 0.866, 0.5]
theta = radians(30)
R = get_rotation_from_axis_angle(axis=skew(w), angle=theta)
print(R)
print(expm(theta*skew(w)))
W, theta = get_axis_angle_from_rotation(R)
print(W, unskew(W), degrees(theta))


def format_transformation(R, p):
    '''
    Format a transformation T (4x4 matrix) from rotation R (3x3 matrix) and position (3 vector).\n
    T = [R p]
        [0 1]
    '''
    p = np.reshape(p, (3, 1))
    T = np.hstack((R, p))
    T = np.vstack((T, np.array([0, 0, 0, 1])))
    return T



#screw matrix [S]
def format_screw(omega, vel):
    '''
    Format a screw [S] (4x4 matrix) from an axis (3 vector) and a velocity (3 vector).\n
    [S] = [[ω̂] v]
          [ 0  0]
    '''
    omega_mat = skew(omega)
    vel = np.reshape(vel, (3, 1))
    S = np.hstack((omega_mat, vel))
    S = np.vstack((S, np.array([0, 0, 0, 0])))
    return S

def get_screw_pitch(omega, vel):
    return vel/omega

def normalize(v):
    mag = np.linalg.norm(v)
    return v/mag

def to_frac(v):
    fracs = []
    try:
        for val in v:
            fracs.append(Fraction(val).limit_denominator())
    except:
        return Fraction(v).limit_denominator()
    return fracs





def cot(theta):
    return 1/tan(theta)

#eqn 3.92, matrix log for rigid body
def get_S_theta(R, p):
    theta = get_theta(R)
    omega = logm(R)
    G_inv = (1/theta)*np.eye(3) - (1/2)*omega + (1/theta - 1/2*cot(theta/2))*(omega@omega)
    v = G_inv @ p
    S = format_screw(omega, v)
    return S, theta

origin = format_transformation(R=np.eye(3), p=np.zeros(3))
