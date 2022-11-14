import numpy as np
from numpy import sin, cos, eye, array, pi
import os, sys
dir = os.path.dirname(__file__)
sys.path.insert(0, dir+'/../')   #allow imports from parent directory
from util import *
from plotcube import plot_cube

np.set_printoptions(precision=2, suppress=True, threshold=5)

import matplotlib.pyplot as plt

L_b = 0.15
L_arm = 0.11945
L_forearm = 0.2115

def forward_kinematics(angles):
    th1, th2, th3, th4 = angles

    # % Robot base (B) -> Shoulder M1 (S1)
    # % rotates around y axis of robot body i.e. rotation axis = [0; 1; 0] 
    Ro_BS1 = array([
        [cos(th1), 0, sin(th1)],
        [0, 1, 0],
        [-sin(th1), 0, cos(th1)],
    ])
    Tr_BS1 = array([0, -L_b/2, 0])
    HTM_BS1 = format_transformation(Ro_BS1, Tr_BS1)


    # % Shoulder M1 (S1) -> Shoulder M2 (S2)
    # % Rotates around the x axis of robot body i.e. [1; 0; 0];
    Ro_S1S2 = array([
        [1, 0, 0],
        [0, cos(th2), -sin(th2)],
        [0, sin(th2), cos(th2)],
    ])
    Tr_S1S2 = array([0.00414, -(0.04803+0.05365), 0])
    HTM_S1S2 = format_transformation(Ro_S1S2, Tr_S1S2)


    HTM_BS2 = HTM_BS1 @ HTM_S1S2

    # % Shoulder M2 (S2) -> Shoulder M3 (S3)
    # % Rotates around the z axis of the robot body i.e. [0; 0; 1];
    Ro_S2S3 = array([
        [cos(th3), -sin(th3), 0],
        [sin(th3), cos(th3), 0],
        [0, 0, 1]
    ])
    Tr_S2S3 = array([0, 0, 0])
    HTM_S2S3 = format_transformation(Ro_S2S3, Tr_S2S3)


    HTM_BS3 = HTM_BS2 @ HTM_S2S3

    # % Shoulder M3 (S3) -> Elbow (E)
    # % rotates around y axis of robot body i.e. rotation axis = [0; 1; 0] 
    Ro_S3E = array([
        [cos(th4), 0, sin(th4)],
        [0, 1, 0],
        [-sin(th4), 0, cos(th4)],
    ])
    Tr_S3E = array([0, 0, -L_arm])
    HTM_S3E = format_transformation(Ro_S3E, Tr_S3E)

    HTM_BE = HTM_BS3 @ HTM_S3E
        
    # % Elbow (E) -> End effector (eF)
    # % no rotation and simple translation
    Ro_EeF = eye(3)
    Tr_EeF = array([-0.05118, 0, -L_forearm])
    HTM_EeF = format_transformation(Ro_EeF, Tr_EeF)
    
    HTM_BeF = HTM_BE @ HTM_EeF

    return HTM_BS1, HTM_BS2, HTM_BS3, HTM_BE, HTM_BeF


fig = plt.figure(figsize=(5, 5))
ax = plt.axes(projection='3d')




for t in np.linspace(0, 1, 100):

    thetas = (sin(5*t), 0, 0, 0)
    T_shoulder, T_shoulder2, T_shoulder3, T_elbow, T_end = forward_kinematics(thetas)
    print(T_end)

    R_shoulder, p_shoulder = extract_R_p_from_transformation(T_shoulder)
    R_shoulder2, p_shoulder2 = extract_R_p_from_transformation(T_shoulder2)
    R_shoulder3, p_shoulder3 = extract_R_p_from_transformation(T_shoulder3)
    R_elbow, p_elbow = extract_R_p_from_transformation(T_elbow)
    R_end, p_end = extract_R_p_from_transformation(T_end)

    points = np.array([p_shoulder, p_shoulder2, p_shoulder3, p_elbow, p_end]).T


    # ax.set_aspect('equal', adjustable='box')
    ax.clear()
    ax.set_xlim(-0.25, 0.25)
    ax.set_ylim(-0.25, 0.25)
    ax.set_zlim(-0.25, 0.25)
    ax.scatter3D(0, 0, 0, c='orange')
    ax.set_title(thetas)
    ax.plot3D(points[0], points[1], points[2])

    cube_definition = [
        (-L_b/2, -L_b/2, -0.1), (0-L_b/2,L_b-L_b/2,0-0.1), (L_b-L_b/2,0-L_b/2,0-0.1), (0-L_b/2,0-L_b/2,0.2-0.1)
    ]
    plot_cube(ax, cube_definition)

    # plt.draw()
    plt.pause(0.02)
    # ax.scatter3D(T_M[0][3], T_M[1][3], T_M[2][3])


plt.show()
