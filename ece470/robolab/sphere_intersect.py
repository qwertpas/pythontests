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