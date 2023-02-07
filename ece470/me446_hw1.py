from util import *
import numpy as np
from numpy import array, pi
from plotcube import *

th1 = pi/4
th2 = pi/6
th3 = -pi/6
L1 = 0.5
L2 = 0.75
L3 = 1

Ro01 = rot_xyz('z', th1)
Tr01 = array([0, 0, 0])
HTM_01 = format_transformation(Ro01, Tr01)

Ro12 = rot_xyz('x', th2)
Tr12 = array([0, 0, L1])
HTM_12 = format_transformation(Ro12, Tr12)

Ro23 = rot_xyz('x', th3)
Tr23 = array([0, L2, 0])
HTM_23 = format_transformation(Ro23, Tr23)

Tr_p = array([0, L3, 0, 1])

P = HTM_01 @ HTM_12 @ HTM_23 @ Tr_p

fig, ax = init_3d_plot(size=(5,5), cube_lim=1)    
plot_origin(ax, lengths=0.25)
plot_HTM(ax, HTM_01, lengths=0.5)
plot_HTM(ax, HTM_01@HTM_12, lengths=0.5)
plot_HTM(ax, HTM_01@HTM_12@HTM_23, lengths=0.5)
plot_point(ax, P)

plt.show()