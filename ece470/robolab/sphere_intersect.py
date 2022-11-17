from matplotlib.widgets import Slider
import numpy as np

import os, sys
dir = os.path.dirname(__file__)
sys.path.insert(0, dir+'/../')   #allow imports from parent directory

from util import *
from plotcube import *

#satyrr dimensions
L_arm = 0.11945     #distance from shoulder to elbow
L_forearm = 0.2115  #distance from elbow to end effector

'''https://en.wikipedia.org/wiki/Line%E2%80%93sphere_intersection'''
def sphere_intersect(line_origin, line_dir, sphere_center, sphere_radius):
    o = np.array(line_origin)
    u = np.array(line_dir)/norm(line_dir)
    c = np.array(sphere_center)
    r = sphere_radius

    determinant = np.dot(u, o-c)**2 - (np.dot(o-c,o-c) - r**2)
    if(determinant < 0):
        print(f"No solution found: determinant={determinant}")
        return 0
    else:
        dist_from_line_origin = -np.dot(u, o-c) + np.sqrt(determinant)
        intersect_pt = o + u*dist_from_line_origin
        return intersect_pt, dist_from_line_origin

'''uses law of cosines to get an angle C of a triangle that has known sides a, b, c'''
def get_angle_of_tri(a, b, c):
    return np.arccos((a**2 + b**2 - c**2)/(2*a*b))

def spherical_fwdk(thetas, order=['x','y','z']):
    '''
    Get the orientation after being transformed by a spherical wrist,
    where thetas are the angles for rotation in the xyz axes in the specified order
    from the base link to farther out in the chain.
    '''
    R = np.eye(3)
    for i in range(3):
        R = R @ rot_xyz(order[i], thetas[i])
    return R

def spherical_invk(R):
    '''
    Find the thetas 1-3 of a spherical wrist to create a desired orientation R
    First convert the yxz frame to zyx frame
    https://www.slideserve.com/marva/ch-3-inverse-kinematics-ch-4-velocity-kinematics
    '''
    shift = np.array([ #convert to whatever frame harvard uses
        [0, 0, 1],
        [1, 0, 0],
        [0, 1, 0]
    ])
    R = shift@R

    print(R)

    s5 = np.sqrt(1 - R[2,2]**2)
    
    #first solution s5 > 0:
    theta4 = np.arctan2(R[0,2], R[1,2]) - pi/2
    theta5 = np.arctan2(R[2,2], s5)
    theta6 = np.arctan2(-R[2,0], R[2,1])
    sol1 = [theta4, theta5, theta6]
    sol1 = [-signed_mod(ang, 2*pi) for ang in sol1]

    #first solution s5 < 0:
    theta4 = np.arctan2(-R[0,2], -R[1,2]) - pi/2
    theta5 = np.arctan2(R[2,2], -s5)
    theta6 = np.arctan2(R[2,0], -R[2,1])
    sol2 = [theta4, theta5, theta6]
    sol2 = [-signed_mod(ang, 2*pi) for ang in sol2]


    return sol1, sol2


fig, ax = init_3d_plot(size=(8,7))    

num_sliders = 3
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

    R = spherical_fwdk(thetas, ['y', 'x', 'z'])
    
    # R = spherical_fwdk(thetas, ['z', 'y', 'z'])

    p0 = np.array([0, 0, -0.1])
    p1 = R@p0

    # print(f"R: {R}")
    print(spherical_invk(R))

    ax.clear()
    plot_frame(ax, R, p1)
    draw_labels(ax)


for slider in sliders:
    slider.on_changed(update)

update()
plt.show()