from numpy import pi, sin, cos
import numpy as np
from sympy import *
import matplotlib.pyplot as plt

''' Question 1(b) Finding HTMs '''

th1, th2, th3 = symbols('θ1 θ2 θ3')
DHs = [
    #a, α, d, θ
    [0, -pi/2, 0.254, th1],
    [0.254, 0, 0, th2],
    [0.254, 0, 0, th3]
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

for i in range(len(HTMs)):
    print(f"HTM{i} in frame 0: ")
    pprint(HTMs[i])

#Create column vector from first 3 values of last column for only position
p_0 = Matrix(HTMs[3].col(-1)[0:3])
print("p_0: ")
pprint(p_0) 

''' Question 1(c) Plotting '''

def plot_arrow(ax, start, dir, color='red', alpha=0.8, arrow_ratio=0.2):
    ax.quiver(
        start[0], start[1], start[2], # starting point of vector
        dir[0], dir[1], dir[2], # directions of vector
        color = color, alpha = alpha,
        arrow_length_ratio=arrow_ratio
    )

def plot_frame(ax, R, p, lengths=0.05, label=""):
    plot_arrow(ax, p, R@np.array([lengths, 0, 0]), color='red')   #x
    plot_arrow(ax, p, R@np.array([0, lengths, 0]), color='green') #y
    plot_arrow(ax, p, R@np.array([0, 0, lengths]), color='blue')  #z
    if(label):
        ax.text(p[0]+lengths/4, p[1]+lengths/4, p[2]+lengths/4, label)

def extract_R_p_from_transformation(T):
    T = np.reshape(T, (4, 4))
    R = T[0:3, 0:3]
    p = T[0:3, 3]
    return R, p

def plot_HTM(ax, HTM, lengths=0.05, label=""):
    R, p = extract_R_p_from_transformation(HTM)
    plot_frame(ax, R, p, lengths, label)

def plot_HTMs(ax, HTMs, links=True, lengths=0.05, label_end=True):
    for i in range(len(HTMs)):
        plot_HTM(ax, HTMs[i], lengths, label=i)
        if(links and i > 0):
            p_start = extract_R_p_from_transformation(HTMs[i-1])[1]
            p_end = extract_R_p_from_transformation(HTMs[i])[1]
            print(p_end)
            plot_arrow(ax, start=p_start, dir=p_end-p_start, arrow_ratio=0, color='grey')
            if(label_end and i == len(HTMs)-1): #label the global position of end effector
                ax.text(p_end[0]+lengths/4, p_end[1]+lengths/4, p_end[2]+lengths, f"({round(p_end[0],2)},{round(p_end[1],2)},{round(p_end[2],2)})")

def plot_origin(ax, lengths=0.05):
    plot_frame(ax, R=np.eye(3), p=np.zeros(3), lengths=lengths)

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
    if(origin):
        plot_origin(ax)
        ax.scatter3D(0, 0, 0, c='orange')
    if(ground):
        plot_ground(ax, width=2*range)
        ax.set_zlim(0, 2*range)
    else:
        ax.set_zlim(-range, range)
    return fig, ax

def plot_CRS(thetas):
    th1_num, th2_num, th3_num = thetas
    fig, ax = init_3d_plot(title=f"thetas: {thetas}", range=0.3)
    plot_HTMs(ax, [HTM.subs({th1: th1_num, th2: th2_num, th3: th3_num}) for HTM in HTMs])
    ax.view_init(azim=120, elev=20)
    plt.show()

# plot_CRS([0, 0, 0])
# plot_CRS([-pi/4, -pi/6, pi/2])
# plot_CRS([pi/2, -pi/3, -pi/3])


''' Question 2(b) '''
from scipy.optimize import fsolve

def CRS_invk(p_des, q_init):
    def error(thetas):
        th1, th2, th3 = thetas
        return [
            0.254*(cos(th2) + cos(th2 + th3))*cos(th1) - p_des[0],
            0.254*(cos(th2) + cos(th2 + th3))*sin(th1) - p_des[1],
            -0.254*sin(th2) - 0.254*sin(th2 + th3) + 0.254 - p_des[2]
        ]
    return fsolve(error, q_init)

#elbow down
solution_thetas = CRS_invk(p_des=[0.3, -0.3, 0.254], q_init=[0, 0, 1])
plot_CRS(solution_thetas)

#elbow up
solution_thetas = CRS_invk(p_des=[0.3, -0.3, 0.254], q_init=[0, 0, -1])
plot_CRS(solution_thetas)
