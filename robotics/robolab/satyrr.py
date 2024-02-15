from matplotlib.widgets import Slider
import numpy as np
from numpy import sin, cos, eye, array, pi
import os, sys
dir = os.path.dirname(__file__)
sys.path.insert(0, dir+'/../')   #allow imports from parent directory
from util import *
from plotcube import *

#[-1.05 -1.05  0.42 -1.21]

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


def plot_satyrr(ax, thetas):
    T_shoulder, T_shoulder2, T_shoulder3, T_elbow, T_end = forward_kinematics(thetas)

    R_shoulder, p_shoulder = extract_R_p_from_transformation(T_shoulder)
    R_shoulder2, p_shoulder2 = extract_R_p_from_transformation(T_shoulder2)
    R_shoulder3, p_shoulder3 = extract_R_p_from_transformation(T_shoulder3)
    R_elbow, p_elbow = extract_R_p_from_transformation(T_elbow)
    R_end, p_end = extract_R_p_from_transformation(T_end)

    points = np.array([p_shoulder, p_shoulder2, p_shoulder3, p_elbow, p_end]).T
    
    ax.scatter3D(0, 0, 0, c='orange')
    ax.set_title(thetas)
    ax.plot3D(points[0], points[1], points[2])

    cube_definition = [
        (-L_b/2, -L_b/2, -0.1), (0-L_b/2,L_b-L_b/2,0-0.1), (L_b-L_b/2,0-L_b/2,0-0.1), (0-L_b/2,0-L_b/2,0.2-0.1)
    ]
    # plot_cube(ax, cube_definition)

    w = 0.02
    transparent_red = (1, 0, 0, 0.2)
    plot_link(ax, R_shoulder, p_shoulder, size=(w, -L_b/2, w), color=transparent_red, long_ax='y')
    plot_link(ax, R_shoulder2, p_shoulder2, size=(w, w, w), color=transparent_red)
    plot_link(ax, R_shoulder3, p_shoulder3, size=(w, w, -L_arm), color=transparent_red)
    elbow_pos_fixed = p_elbow + R_elbow@np.array([-0.05118, 0, 0])
    plot_link(ax, R_elbow, elbow_pos_fixed, size=(w, w, -L_forearm), color=transparent_red)

    plot_link(ax, np.eye(3), (0, 0, -L_b), size=(L_b/2, L_b/2, 2*L_b), color=transparent_red) #body

    plot_frame(ax, R_end, p_end)


if __name__ == "__main__":
    fig = plt.figure(figsize=(8, 8))
    ax = plt.axes(projection='3d')


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
        ax.clear()
        ax.set_xlim(-0.25, 0.25)
        ax.set_ylim(-0.25, 0.25)
        ax.set_zlim(-0.25, 0.25)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plot_satyrr(ax, thetas)

    for slider in sliders:
        slider.on_changed(update)


    update()
    plt.show()