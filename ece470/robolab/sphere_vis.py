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


fig, ax = init_3d_plot()


num_sliders = 7
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
    s = []
    for slider in sliders:
        s.append(slider.val)

    p_joy_end = (s[0], s[1], s[2])
    R_joy_end = get_rotation_from_axis_angle(axis=skew(normalize([s[3], s[4], s[5]])), angle=s[6])



    ang_joy_end = R_joy_end[:,1]

    p_joy_elb = p_joy_end - L_forearm * ang_joy_end

    p_sat_elb, _ = sphere_intersect(
        line_origin=p_joy_end,
        line_dir=p_joy_end,
        sphere_center=(0,0,0),
        sphere_radius=L_arm
    )

    p_sat_end = p_sat_elb + ang_joy_end*L_forearm






    ax.clear()
    plot_link(ax, R_joy_end, p_joy_elb, size=(0.004, 0.004, L_forearm), color='red')
    plot_link(ax, R_joy_end, p_sat_elb, size=(0.004, 0.004, L_forearm), color='blue')

    plot_link(ax, R_joy_end, p_joy_elb, size=(0.005, 0.005, 0.005), color='black')
    plot_link(ax, R_joy_end, p_sat_elb, size=(0.005, 0.005, 0.005), color='black')

    # ax.scatter(p_joy_elb[0], p_joy_elb[1], p_joy_elb[2])
    # ax.scatter(p_joy_end[0], p_joy_end[1], p_joy_end[2])
    draw_labels(ax, cube_lim=0.25)


for slider in sliders:
    slider.on_changed(update)

update()
plt.show()