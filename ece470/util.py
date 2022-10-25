from multiprocessing.connection import wait
from tkinter import W
import numpy as np
from numpy import sin, cos, tan, pi, arccos, arcsin, sqrt, radians, degrees
from scipy.linalg import expm, logm
import matplotlib.pyplot as plt
from fractions import Fraction

np.set_printoptions(precision=3, suppress=True)


def cross(a:np.ndarray,b:np.ndarray)->np.ndarray:
    '''
    workaround for bug with np.cross() that makes code unreachable
    '''
    return np.cross(a,b)

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
    epsilon = 1e-3
    if(np.linalg.norm(R - np.eye(3)) < epsilon):
        theta = 0 #axis W is undefined
        W = np.nan
    elif(np.abs(np.trace(R) + 1) < epsilon):
        theta = pi
        print("there are 6 solutions for W")
        w = 1/sqrt(2*(1+R[2,2])) * np.array([R[0,2], R[1,2], 1+R[2,2]])
        # w = 1/sqrt(2*(1+R[1,1])) * np.array([R[0,1], 1+R[1,1], R[2,1]])
        # w = 1/sqrt(2*(1+R[0,0])) * np.array([1+R[0,0], R[1,0], R[2,0]])
        W = skew(w)
    else:
        theta = arccos((np.trace(R) - 1) / 2)   #(3.54)
        W = (R - R.T)/ (2 * sin(theta))         #(3.53)
    axis = W
    angle = theta
    return axis, angle


# w = [0, 0.866, 0.5]
# theta = radians(180)
# R = get_rotation_from_axis_angle(axis=skew(w), angle=theta)
# print(R)
# W, theta = get_axis_angle_from_rotation(R)
# print(W, unskew(W), degrees(theta))


def format_transformation(R, p):
    '''
    Format a transformation T (4x4 matrix) from rotation R (3x3 matrix) and position (3 vector).\n
    T = [R p]
        [0 1]
    (Modern Robotics 3.62)
    '''
    p = np.reshape(p, (3, 1))
    T = np.hstack((R, p))
    T = np.vstack((T, np.array([0, 0, 0, 1])))
    return T

def extract_R_p_from_transformation(T):
    '''
    Extract rotation R (3x3 matrix) and displacement p (3 vector) from a transformation (4x4 matrix)
    '''
    T = np.reshape(T, (4, 4))
    R = T[0:3, 0:3]
    p = T[0:3, 3]
    return R, p

def transform_point(T, x):
    '''
    Evaluates Tx = Rx + p = T@[x,1], where T = [R, p], a rotation R and displacement p. Left multiplication means rotate then displace.
    Appending a 1 to the 3 vector puts it in homogenous coordinates so it is compatible with transformation matrices
    T: 4x4 transformation matrix
    x: 3x3 vector position in space
    (Modern Robotics 3.65)
    '''
    x = np.array(x).flatten()
    x_homogeneous = np.concatenate((x, [1]))
    Tx_homogenous = T @ x_homogeneous
    return Tx_homogenous[0:-1]

def adjoint(T):
    '''
    Get [Adj_Tsb] (6x6 matrix) which converts twists in the b frame to twists in the s frame (as 6 vectors): twist_s = [Adj_Tsb] @ twist_b  
    (Modern Robotics 3.83)
    '''
    T = np.array(T).reshape((4, 4))
    R, p = extract_R_p_from_transformation(T)
    P = skew(p)
    top_3_rows = np.hstack((R, np.zeros((3,3))))
    low_3_rows = np.hstack((P@R, R))
    Adj_Tsb = np.vstack((top_3_rows, low_3_rows))
    return Adj_Tsb

def format_twist_matrix(w, v):
    '''
    Format a twist [V] (4x4 matrix) from angular velocity w (3 vector) and velocity (3 vector).\n
    w is the axis of rotation scaled by angular velocity.
    w and v can be in body or space frame, and the screw will be in the corresponding frame.
    [V] = [[ω] v]
          [ 0  0]
    (Modern Robotics 3.81)
    '''
    W = skew(w)
    v = np.reshape(v, (3, 1))
    Twist = np.hstack((W, v))
    Twist = np.vstack((Twist, np.array([0, 0, 0, 0])))
    return Twist

def extract_twist_vector_from_matrix(Twist):
    '''
    Extract the twist vector (6 vector) consisting of [ω v] where ω and v are angular and linear velocity (both 3 vectors).  
    '''
    Twist = np.reshape(Twist, (4, 4))
    W = Twist[0:3,0:3]
    w = unskew(W)
    v = Twist[0:3, 3]
    twist = np.concatenate((w, v))
    return twist

# #example 3.23, the tricycle
# R_sb = np.array([ #car frame rotation in space frame
#     [-1, 0, 0],
#     [0, 1, 0],
#     [0, 0, -1]
# ])
# p_sb = np.array((4, 0.4, 0)) #position of car in space frame
# r_s = np.array((2, -1, 0))
# T_sb = format_transformation(R_sb, p_sb)
# r_b = transform_point(np.linalg.inv(T_sb), r_s)
# w_s = np.array((0, 0, 2))
# w_b = np.array((0, 0, -2))
# v_s = cross(r_s, w_s)       #v = r x ω
# v_b = cross(r_b, w_b)
# twist_s = np.concatenate((w_s, v_s))
# twist_b = np.concatenate((w_b, v_b))
# print(f"twist_s: {twist_s}, twist_b: {twist_b}")
# Adj_Tsb = adjoint(T_sb)
# print(Adj_Tsb @ twist_b)    #show that Adj can convert body twist to space twist




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
