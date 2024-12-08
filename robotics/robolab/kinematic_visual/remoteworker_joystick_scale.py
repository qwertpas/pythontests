from matplotlib.widgets import Slider
import numpy as np
from numpy import sin, cos, eye, array, pi

import os, sys
dir = os.path.dirname(__file__)
sys.path.insert(0, dir+'/../')   #allow imports from parent directory

from util import *
from plotcube import *
from remoteworker import plot_satyrr



np.set_printoptions(precision=3, suppress=True, threshold=5)

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

L_shoulder_from_body = 0.2105 #middle to shoulder pivot (to the right)
L_shoulder_x = 0.1035 #forward
L_shoulder_y = 0.168 #to the right
# L_shoulder_z = 0.168 #downwards
L_arm = 0.313 #upper arm length
L_forearm = 0.339 # lower arm length
L_hand = 0.078 #width of the hand that goes inwards
joy_right_shoulder = np.array([0, -L_shoulder_from_body-L_shoulder_y, 0])


L_sat_forearm = 0.2115
L_sat_arm = 0.290
L_sat_shoulder_from_body = 0.120
sat_right_shoulder = np.array([0, -L_sat_shoulder_from_body, 0])


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

def forward_kinematics(angles, is_right):
    th1, th2, th3, th4 = angles
    if is_right:
        pol = 1
    else:
        pol = -1
 
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
    # Tr_S2S3 = array([0, -L_shoulder_y, -L_shoulder_z])
    Tr_S2S3 = array([L_shoulder_x, -L_shoulder_y, 0])
    HTM_S2S3 = format_transformation(Ro_S2S3, Tr_S2S3)


    HTM_BS3 = HTM_BS2 @ HTM_S2S3

    # % Shoulder M3 (S3) -> Elbow (E), Elbow
    # % rotates around y axis of robot body i.e. rotation axis = [0; 1; 0] 
    Ro_S3E = rotMatrix('y', -th4)
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

p_joy_elb = np.zeros(3)

fig, ax = init_3d_plot(size=(10,9), cube_lim=0.05)    


def plot_arm(T_shoulder, T_shoulder2, T_shoulder3, T_elbow, T_end, thetas):
    R_shoulder, p_shoulder = extract_R_p_from_transformation(T_shoulder)
    R_shoulder2, p_shoulder2 = extract_R_p_from_transformation(T_shoulder2)
    R_shoulder3, p_shoulder3 = extract_R_p_from_transformation(T_shoulder3)
    R_elbow, p_elbow = extract_R_p_from_transformation(T_elbow)
    R_end, p_end = extract_R_p_from_transformation(T_end)

    print("human hand: \n", T_end)

    # p_sat_elb = satyrr_joystick_invk(R_end, p_end)
    ang_joy_end = -R_end[:,2]

    p_joy_elb = p_end - L_forearm * ang_joy_end - np.array([L_shoulder_x,L_hand-L_shoulder_from_body-L_shoulder_y,0])
    # p_joy_elb = p_joy_end - L_sat_forearm * ang_joy_end
    # p_joy_elb = p_joy_end 
    p_sat_elb = L_sat_arm * normalize(p_joy_elb)
    # print('p_sat_elb', p_sat_elb)

    plot_frame(ax, np.eye(3), p_joy_elb, lengths=0.5)


    ax.clear()
    w = 0.02

    #joystick
    plot_link(ax, R_shoulder2, p_shoulder, size=(w, -L_shoulder_y, w), long_ax='y')
    plot_link(ax, R_shoulder2, p_shoulder3, size=(-L_shoulder_x, w, w), long_ax='x')
    plot_link(ax, R_shoulder3, p_shoulder3, size=(w, w, -L_arm))
    plot_link(ax, R_elbow, p_elbow, size=(w, w, -L_forearm))
    plot_link(ax, R_end, p_end, size=(w, w, w))

    #joystick body
    plot_link(ax, np.eye(3), (0, 0, -0.25), size=(0.1, L_shoulder_from_body, 0.5), color=(0, 0, 1, 0.1))

    #satyrr arm
    # plot_frame(ax, R_end, p_sat_elb + sat_right_shoulder)

    # p_sat_elb = rot_xyz('x', radians(60)) @ p_sat_elb

    Rz = -normalize(p_sat_elb) 
    plot_arrow(ax, sat_right_shoulder+p_sat_elb, R_end[:,1]*0.1, color='gray')
    Ry = normalize(R_end[:,1] - Rz*np.dot(R_end[:,1], Rz)) 
    Rx = normalize(cross(Ry, Rz))
    # R_sphere = np.hstack((Rx.reshape((3,1)), Ry.reshape((3,1)), Rz.reshape((3,1))))
    R_sphere = np.hstack((Rx.reshape((3,1)), Ry.reshape((3,1)), Rz.reshape((3,1))))
    plot_frame(ax, R_sphere, sat_right_shoulder+p_sat_elb, lengths=0.1)

    result_thetas = np.zeros(4)
    result_thetas[0:3] = spherical_invk(R_sphere)[0]
    # result_thetas[3] = -ang_betw(R_end[:,2], Rz, axis=Ry)    #angle between -R_end[:,2] and Rz
    result_thetas[3] = -thetas[3]    #angle between -R_end[:,2] and Rz

    # if(result_thetas[3] > -0.41):
    #     result_thetas[3] -= 0.41*2

    # print("R_sphere ", R_sphere)
    # print("result_thetas ", result_thetas)

    # print(degrees(result_thetas))

    # plot_frame(ax, R_end, p_joy_elb)
    plot_frame(ax, np.eye(3), sat_right_shoulder)

    plot_satyrr(ax, result_thetas)

    Jc_R = np.zeros((3,4))
    th1_arm = result_thetas[0]
    th2_arm = result_thetas[1]
    th3_arm = result_thetas[2]
    th4_arm = result_thetas[3]
    Jc_R[0][0] = 0.21144060406275340674442375643594*sin(th4_arm)*(cos(th3_arm)*sin(th1_arm) - 1.0*cos(th1_arm)*sin(th2_arm)*sin(th3_arm)) - 0.00414*sin(th1_arm) - 0.11945*cos(th1_arm)*cos(th2_arm) + 0.046167919236689778546001150516531*cos(th4_arm)*(cos(th3_arm)*sin(th1_arm) - 1.0*cos(th1_arm)*sin(th2_arm)*sin(th3_arm)) - 0.21144060406275340674442375643594*cos(th1_arm)*cos(th2_arm)*cos(th4_arm) + 0.046167919236689778546001150516531*cos(th1_arm)*cos(th2_arm)*sin(th4_arm);
    Jc_R[0][1] = 0.11945*sin(th1_arm)*sin(th2_arm) + 0.21144060406275340674442375643594*cos(th4_arm)*sin(th1_arm)*sin(th2_arm) - 0.046167919236689778546001150516531*sin(th1_arm)*sin(th2_arm)*sin(th4_arm) - 0.046167919236689778546001150516531*cos(th2_arm)*cos(th4_arm)*sin(th1_arm)*sin(th3_arm) - 0.21144060406275340674442375643594*cos(th2_arm)*sin(th1_arm)*sin(th3_arm)*sin(th4_arm);
    Jc_R[0][2] = 0.21144060406275340674442375643594*sin(th4_arm)*(cos(th1_arm)*sin(th3_arm) - 1.0*cos(th3_arm)*sin(th1_arm)*sin(th2_arm)) + 0.046167919236689778546001150516531*cos(th4_arm)*(cos(th1_arm)*sin(th3_arm) - 1.0*cos(th3_arm)*sin(th1_arm)*sin(th2_arm));
    Jc_R[0][3] = 0.046167919236689778546001150516531*sin(th4_arm)*(cos(th1_arm)*cos(th3_arm) + sin(th1_arm)*sin(th2_arm)*sin(th3_arm)) - 0.21144060406275340674442375643594*cos(th4_arm)*(cos(th1_arm)*cos(th3_arm) + sin(th1_arm)*sin(th2_arm)*sin(th3_arm)) + 0.21144060406275340674442375643594*cos(th2_arm)*sin(th1_arm)*sin(th4_arm) + 0.046167919236689778546001150516531*cos(th2_arm)*cos(th4_arm)*sin(th1_arm);

    Jc_R[1][0] = 0;
    Jc_R[1][1] = 0.11945*cos(th2_arm) + 0.21144060406275340674442375643594*cos(th2_arm)*cos(th4_arm) - 0.046167919236689778546001150516531*cos(th2_arm)*sin(th4_arm) + 0.046167919236689778546001150516531*cos(th4_arm)*sin(th2_arm)*sin(th3_arm) + 0.21144060406275340674442375643594*sin(th2_arm)*sin(th3_arm)*sin(th4_arm);
    Jc_R[1][2] = -0.046167919236689778546001150516531*cos(th2_arm)*cos(th3_arm)*cos(th4_arm) - 0.21144060406275340674442375643594*cos(th2_arm)*cos(th3_arm)*sin(th4_arm);
    Jc_R[1][3] = 0.046167919236689778546001150516531*cos(th2_arm)*sin(th3_arm)*sin(th4_arm) - 0.21144060406275340674442375643594*sin(th2_arm)*sin(th4_arm) - 0.046167919236689778546001150516531*cos(th4_arm)*sin(th2_arm) - 0.21144060406275340674442375643594*cos(th2_arm)*cos(th4_arm)*sin(th3_arm);

    Jc_R[2][0] = 0.11945*cos(th2_arm)*sin(th1_arm) - 0.00414*cos(th1_arm) + 0.046167919236689778546001150516531*cos(th4_arm)*(cos(th1_arm)*cos(th3_arm) + sin(th1_arm)*sin(th2_arm)*sin(th3_arm)) + 0.21144060406275340674442375643594*sin(th4_arm)*(cos(th1_arm)*cos(th3_arm) + sin(th1_arm)*sin(th2_arm)*sin(th3_arm)) - 0.046167919236689778546001150516531*cos(th2_arm)*sin(th1_arm)*sin(th4_arm) + 0.21144060406275340674442375643594*cos(th2_arm)*cos(th4_arm)*sin(th1_arm);
    Jc_R[2][1] = 0.11945*cos(th1_arm)*sin(th2_arm) - 0.046167919236689778546001150516531*cos(th1_arm)*sin(th2_arm)*sin(th4_arm) + 0.21144060406275340674442375643594*cos(th1_arm)*cos(th4_arm)*sin(th2_arm) - 0.046167919236689778546001150516531*cos(th1_arm)*cos(th2_arm)*cos(th4_arm)*sin(th3_arm) - 0.21144060406275340674442375643594*cos(th1_arm)*cos(th2_arm)*sin(th3_arm)*sin(th4_arm);
    Jc_R[2][2] = -0.046167919236689778546001150516531*cos(th4_arm)*(sin(th1_arm)*sin(th3_arm) + cos(th1_arm)*cos(th3_arm)*sin(th2_arm)) - 0.21144060406275340674442375643594*sin(th4_arm)*(sin(th1_arm)*sin(th3_arm) + cos(th1_arm)*cos(th3_arm)*sin(th2_arm));
    Jc_R[2][3] = 0.21144060406275340674442375643594*cos(th4_arm)*(cos(th3_arm)*sin(th1_arm) - 1.0*cos(th1_arm)*sin(th2_arm)*sin(th3_arm)) - 0.046167919236689778546001150516531*sin(th4_arm)*(cos(th3_arm)*sin(th1_arm) - 1.0*cos(th1_arm)*sin(th2_arm)*sin(th3_arm)) + 0.046167919236689778546001150516531*cos(th1_arm)*cos(th2_arm)*cos(th4_arm) + 0.21144060406275340674442375643594*cos(th1_arm)*cos(th2_arm)*sin(th4_arm);

    # print(Jc_R)

    P = np.eye(4,4) - np.linalg.pinv(Jc_R)@Jc_R

    tau_task2 = np.array([1,2,3,4])
    
    # print("projectedtau2", P@tau_task2)
    # print("should be 0", Jc_R@(P@tau_task2))
    # for i in range(len(result_thetas)): #bar plots for joint angles
    #     plot_link(ax, np.eye(3), p=(0.3+0.05*i, 0, 0), size=(w,w,0.4/pi*result_thetas[i]))


    # draw sphere
    r = L_sat_arm
    u, v = np.mgrid[0:2*np.pi:10j, 0:np.pi:10j]
    x = np.cos(u)*np.sin(v)*r
    y = np.sin(u)*np.sin(v)*r + L_sat_shoulder_from_body
    z = np.cos(v)*r
    # ax.plot_wireframe(x, y, z, color="gray")
    # ax.plot_wireframe(x, -y, z, color=(0, 1, 0, 0.3))

    draw_labels(ax, cube_lim=0.5)
    ax.set_title(f"Satyrr θ: {result_thetas}")

num_sliders = 4
sliders = []
for i in range(num_sliders):
    height = 0.05
    space = height * num_sliders
    fig.subplots_adjust(bottom=space)
    slider_ax = fig.add_axes([0.25, space - i*height, 0.65, 0.03])
    slider = Slider(
        ax=slider_ax,
        label=f"Joystick θ{i}",
        valmin=-pi,
        valmax=pi,
        valinit=0,
    ) 
    sliders.append(slider)

def update(val=0):
    thetas = []
    for slider in sliders:
        thetas.append(slider.val)
    # thetas = [0.826, -0.594, -0.846, -1.358]
    T_shoulder, T_shoulder2, T_shoulder3, T_elbow, T_end = forward_kinematics(thetas, is_right=True)
    plot_arm(T_shoulder, T_shoulder2, T_shoulder3, T_elbow, T_end, thetas)

for slider in sliders:
    slider.on_changed(update)

update()
plt.show()