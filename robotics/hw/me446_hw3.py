from numpy import pi, sin, cos
import numpy as np
from sympy import *
import matplotlib.pyplot as plt

''' HW2 Question 1(b) Finding HTMs '''

th1, th2, th3 = symbols('θ1 θ2 θ3')
L = 0.254
DHs = [
    #a, α, d, θ
    [0, -pi/2, L, th1],
    [L, 0, 0, th2],
    [L, 0, 0, th3]
]
HTMs = [Matrix([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])] #start with base frame, will append more

for i in range(len(DHs)):
    DH = DHs[i]
    a, alpha, d, theta = DH
    Rot_th = Matrix([
        [cos(theta), -sin(theta), 0, 0],
        [sin(theta), cos(theta), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    Trans_d = Matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, d],
        [0, 0, 0, 1]
    ])
    Trans_a = Matrix([
        [1, 0, 0, a],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    Rot_alpha = Matrix([
        [1, 0, 0, 0],
        [0, cos(alpha), -sin(alpha), 0],
        [0, sin(alpha), cos(alpha), 0],
        [0, 0, 0, 1]
    ])
    HTM = Rot_th @ Trans_d @ Trans_a @ Rot_alpha
    HTMs.append(simplify(HTMs[-1] @ HTM))


''' HW 2 Question 1(c) Plotting '''
def plot_arrow(ax, start, dir, color='red', alpha=0.8, arrow_ratio=0.2):
    ax.quiver(
        start[0], start[1], start[2], # starting point of vector
        dir[0], dir[1], dir[2], # directions of vector
        color = color, alpha = alpha,
        arrow_length_ratio=arrow_ratio
    )

def extract_R_p(T):
    T = np.reshape(T, (4, 4))
    R = T[0:3, 0:3]
    p = T[0:3, 3]
    return R, p

def plot_HTM(ax, HTM, lengths=0.05, label=""):
    R, p = extract_R_p(HTM)
    ax.scatter3D(p[0], p[1], p[2], c='black')

def plot_HTMs(ax, HTMs, links=True, lengths=0.05, label_end=True):
    for i in range(len(HTMs)):
        plot_HTM(ax, HTMs[i], lengths, label=i)
        if(links and i > 0):
            p_start = extract_R_p(HTMs[i-1])[1]
            p_end = extract_R_p(HTMs[i])[1]
            plot_arrow(ax, start=p_start, dir=p_end-p_start, arrow_ratio=0, color='grey')
            if(label_end and i == len(HTMs)-1): #label the global position of end effector
                ax.text(p_end[0]+lengths/4, p_end[1]+lengths/4, p_end[2]+lengths, f"({round(p_end[0],3)},{round(p_end[1],3)},{round(p_end[2],3)})")


def plot_ground(ax, width=1):
    xx, yy = np.meshgrid(np.linspace(-width/2, width/2, 2), np.linspace(-width/2, width/2, 2))
    z = np.zeros((2, 2))
    ax.plot_surface(xx, yy, z, alpha=0.5)

def init_3d_plot(title='', range=1, figsize=(8,8), origin=True, ground=True):
    fig = plt.figure(figsize=figsize)
    fig.tight_layout()
    ax = plt.axes(projection='3d')
    ax.set_title(title)
    ax.set_box_aspect([1,1,1])
    ax.set_xlim(-range, range)
    ax.set_ylim(-range, range)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    if(origin):
        ax.scatter3D(0, 0, 0, c='orange')
    if(ground):
        plot_ground(ax, width=2*range)
        ax.set_zlim(0, 2*range)
    else:
        ax.set_zlim(-range, range)
    return fig, ax

''' HW3 1(c) '''
#End-effector jacobian
p = Matrix([
    L*cos(th1)*(cos(th2 + th3) + cos(th2)),
    L*sin(th1)*(cos(th2 + th3) + cos(th2)),
    L*(1 - sin(th2) - sin(th2 + th3))
])
dp_dq1 = diff(p, th1)
dp_dq2 = diff(p, th2)
dp_dq3 = diff(p, th3)
Jv = simplify(dp_dq1.row_join(dp_dq2).row_join(dp_dq3))  

#Plot arm and velocity vector
def plot_CRS(thetas, theta_dots):
    th1_num, th2_num, th3_num = thetas
    fig, ax = init_3d_plot(title=f"thetas: {thetas}, thetadots: {theta_dots}", range=0.3)
    plot_HTMs(ax, [HTM.subs({th1: th1_num, th2: th2_num, th3: th3_num}) for HTM in HTMs])

    subs_dict = {th1:th1_num, th2:th2_num, th3:th3_num}
    end_pos = extract_R_p(HTMs[-1].subs(subs_dict))[1]
    Jv_num = Jv.subs(subs_dict)
    end_vel = Jv_num @ Matrix(theta_dots)
    plot_arrow(ax, start=end_pos, dir=end_vel)

    ax.view_init(azim=120, elev=20)
    plt.show()

''' HW3 1(d) '''
# plot_CRS(thetas=[0, 0, 0], theta_dots=[0, 0, 1])
# plot_CRS(thetas=[0, 0, pi/2], theta_dots=[1, 1, 0])

''' HW3 1(e) '''
th1_num, th2_num, th3_num = [pi/6, -pi/4, pi/2]
theta_dots = [0, 1, 1]
Jv_inv = Jv.subs({th1:th1_num, th2:th2_num, th3:th3_num}).inv()
q_dot = Jv_inv @ Matrix(theta_dots)
pprint(q_dot) # joint velocities to reach desired end velocity


