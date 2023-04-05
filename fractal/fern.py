from matplotlib.widgets import Slider, Button
import numpy as np
import matplotlib.pyplot as plt
from random import random

params = [
    0, 0, 0, 0.16,
    0.85, 0.04, -0.04, 0.85,
    0.2, -0.26, 0.24, 0.22,
    -0.15, 0.28, 0.26, 0.24,
    0, 0,
    0, 1.6,
    0, 1.6,
    0, 0.44,
    0.05,
    0.8,
    0.05,
    0.09,
    0
]

width = 14
height = 8
fig, ax = plt.subplots(figsize=(width, height))
fig.subplots_adjust(left=0.2)

sliders = []
for i in range(len(params)):
    if(i != 28):
        new_slider_ax = fig.add_axes([0.1, 0.8-i*height/len(params)*0.1, 0.1, 0.01])
    else:
        new_slider_ax = fig.add_axes([0.1, 0.8-i*height/len(params)*0.1, 0.8, 0.01])
    
    new_slider_ax.axis("off")

    new_slider = Slider(
        ax=new_slider_ax,
        label=f"params[{i}]",
        valmin= -abs(2*params[i]-1),
        valmax= +abs(2*params[i]+1),
        valinit= params[i],    
    ) 
    sliders.append(new_slider)

button_ax = fig.add_axes([0.1, 0.9, height/len(params), 0.02])
button = Button(button_ax, "Reset", color=(0.5,0.5,0.9), hovercolor=(0.5,0.5,0.9))
def reset_params(val=0):
    for i in range(len(sliders)):
        sliders[i].set_val(params[i])
button.on_clicked(reset_params)

# Initialize the arrays to store the points
n = 2000
n_ferns = 14
ferns = []
for i in range(n_ferns):
    color = np.full((n, 3), (0.2+0.2*random(), 0.5+0.3*random(), 0.1))
    ferns.append(ax.scatter(np.zeros(n), np.zeros(n), s=0.1, c=color))
ax.set_title("Barnsley Fern")
ax.axis("off")


def update(val=0):
    # Define the parameters for the Barnsley fern algorithm
    # Matrices


    # Define the function to generate the Barnsley fern
    def barnsley_fern(n, j):
        x = np.zeros(n)
        y = np.zeros(n)
        x[0] = 0
        y[0] = 0

        #adjust based on which fern j
        p = [slider.val for slider in sliders]
        p[5] += 0.02*np.sin(p[28]*10 - 0.1*j)
        p[6] += 0.1*random()*np.sin(p[28]*20 - 0.8*j)
        p[7] += 0.02*np.sin(p[28]*30 - 0.5*j)

        a = np.array([[p[0], p[1]], [p[2], p[3]]])
        b = np.array([[p[4], p[5]],[p[6],p[7]]])
        c = np.array([[p[8], p[9]],[p[10],p[11]]])
        d = np.array([[p[12], p[13]],[p[14],p[15]]])

        # Vectors
        a1 = np.array([p[16], p[17]])
        b1 = np.array([p[18], p[19]])
        c1 = np.array([p[20], p[21]])
        d1 = np.array([p[22], p[23]])

        # Define the probability distributions for each transformation
        p1 = p[24]
        p2 = p[25]
        p3 = p[26]
        p4 = p[27]

        # Iterate the Barnsley fern algorithm
        for i in range(1, n):
            r = np.random.rand()
            if r < p1:
                x[i], y[i] = np.matmul(a, [x[i-1], y[i-1]]) + a1
            elif r < p1 + p2:
                x[i], y[i] = np.matmul(b, [x[i-1], y[i-1]]) + b1
            elif r < p1 + p2 + p3:
                x[i], y[i] = np.matmul(c, [x[i-1], y[i-1]]) + c1
            else:
                x[i], y[i] = np.matmul(d, [x[i-1], y[i-1]]) + d1

        # Return the points
        return x, y

    # Generate the Barnsley fern and display it using Matplotlib
    for i in range(n_ferns):
        x, y = barnsley_fern(n, i)
        x = x + i*1 - 5
        ferns[i].set_offsets(np.column_stack((x, y)))
    ax.set_xlim(-10,10)
    ax.set_ylim(0,20)

for slider in sliders:
    slider.on_changed(update)

update()
plt.show()

