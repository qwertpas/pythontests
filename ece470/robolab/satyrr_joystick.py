from matplotlib.widgets import Slider
import numpy as np
from numpy import sin, cos, eye, array, pi

import os, sys
dir = os.path.dirname(__file__)
sys.path.insert(0, dir+'/../')   #allow imports from parent directory

from util import *
from plotcube import plot_cube



np.set_printoptions(precision=2, suppress=True, threshold=5)

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

L_shoulder_from_body = 0.2105 #middle to shoulder pivot (to the right)
L_shoulder_y = 0.1035 #to the right
L_shoulder_z = 0.168 #downwards
L_arm = 0.313 #upper arm length
L_forearm = 0.339 # lower arm length
L_hand = 0.078 #width of the hand that goes inwards

def rotMatrix(axis, theta):
    if(axis == 'x'):
        return array([
            [1, 0, 0],
            [0, cos(theta), -sin(theta)],
            [0, sin(theta), cos(theta)],
        ])
    if(axis == 'y'):
        return array([
            [cos(theta), 0, sin(theta)],
            [0, 1, 0],
            [-sin(theta), 0, cos(theta)],
        ])
    if(axis == 'z'):
        return array([
            [cos(theta), -sin(theta), 0],
            [sin(theta), cos(theta), 0],
            [0, 0, 1]
        ])

def forward_kinematics(angles):
    th1, th2, th3, th4 = angles
 
    # % Robot base (B) -> Shoulder M1 (S1) on the back
    # % rotates around z axis of robot body i.e. rotation axis = [0; 1; 0] 
    Ro_BS1 = rotMatrix('z', th1)
    Tr_BS1 = array([0, -L_shoulder_from_body, 0])
    HTM_BS1 = format_transformation(Ro_BS1, Tr_BS1)


    # % Shoulder M1 (S1) -> Shoulder M2 (S2) Same point as before, on the back
    # % Rotates around the z axis of robot body i.e. [1; 0; 0];
    Ro_S1S2 = rotMatrix('x', th2)
    Tr_S1S2 = array([0, 0, 0])
    HTM_S1S2 = format_transformation(Ro_S1S2, Tr_S1S2)


    HTM_BS2 = HTM_BS1 @ HTM_S1S2

    # % Shoulder M2 (S2) -> Shoulder M3 (S3), Outer shoulder
    # % Rotates around the y axis of the robot body i.e. [0; 0; 1];
    Ro_S2S3 = rotMatrix('y', th3)
    Tr_S2S3 = array([0, -L_shoulder_y, -L_shoulder_z])
    HTM_S2S3 = format_transformation(Ro_S2S3, Tr_S2S3)


    HTM_BS3 = HTM_BS2 @ HTM_S2S3

    # % Shoulder M3 (S3) -> Elbow (E), Elbow
    # % rotates around y axis of robot body i.e. rotation axis = [0; 1; 0] 
    Ro_S3E = rotMatrix('y', th4)
    Tr_S3E = array([0, 0, -L_arm])
    HTM_S3E = format_transformation(Ro_S3E, Tr_S3E)

    HTM_BE = HTM_BS3 @ HTM_S3E
        
    # % Elbow (E) -> End effector (eF)
    # % no rotation and simple translation
    Ro_EeF = eye(3)
    Tr_EeF = array([0, L_hand, -L_forearm])
    HTM_EeF = format_transformation(Ro_EeF, Tr_EeF)
    
    HTM_BeF = HTM_BE @ HTM_EeF

    return HTM_BS1, HTM_BS2, HTM_BS3, HTM_BE, HTM_BeF


fig = plt.figure(figsize=(5, 5))
ax = plt.axes(projection='3d')

def plot_arm(T_shoulder, T_shoulder2, T_shoulder3, T_elbow, T_end):
    R_shoulder, p_shoulder = extract_R_p_from_transformation(T_shoulder)
    R_shoulder2, p_shoulder2 = extract_R_p_from_transformation(T_shoulder2)
    R_shoulder3, p_shoulder3 = extract_R_p_from_transformation(T_shoulder3)
    R_elbow, p_elbow = extract_R_p_from_transformation(T_elbow)
    R_end, p_end = extract_R_p_from_transformation(T_end)

    points = np.array([p_shoulder, p_shoulder2, p_shoulder3, p_elbow, p_end]).T

    # ax.set_aspect('equal', adjustable='box')
    ax.clear()
    ax.set_xlim(-0.5, 0.5)
    ax.set_ylim(-0.5, 0.5)
    ax.set_zlim(-0.5, 0.5)
    ax.scatter3D(0, 0, 0, c='orange')
    ax.plot3D(points[0], points[1], points[2])
    L_b = L_shoulder_from_body*2
    cube_definition = [
        (-L_b/2, -L_b/2, -0.1), (0-L_b/2,L_b-L_b/2,0-0.1), (L_b-L_b/2,0-L_b/2,0-0.1), (0-L_b/2,0-L_b/2,0.2-0.1)
    ]
    plot_cube(ax, cube_definition)

    # ax.scatter3D(T_M[0][3], T_M[1][3], T_M[2][3])

num_sliders = 4
sliders = []
for i in range(num_sliders):
    height = 0.05
    space = height * num_sliders
    fig.subplots_adjust(bottom=space)
    slider_ax = fig.add_axes([0.25, space - i*height, 0.65, 0.03])
    slider = Slider(
        ax=slider_ax,
        label=f"theta {i}",
        valmin=-pi,
        valmax=pi,
        valinit=0,
    ) 
    sliders.append(slider)

def update(val=0):
    thetas = []
    for slider in sliders:
        thetas.append(slider.val)
    T_shoulder, T_shoulder2, T_shoulder3, T_elbow, T_end = forward_kinematics(thetas)
    plot_arm(T_shoulder, T_shoulder2, T_shoulder3, T_elbow, T_end)

for slider in sliders:
    slider.on_changed(update)

update()
plt.show()