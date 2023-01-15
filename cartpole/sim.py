import matplotlib.pyplot as plt
import numpy as np
from numpy import sin, cos, radians
from scipy.integrate import RK45

K = np.array([-1, -0.1, 30, 1])

def cartpole(t, state):
    l = 0.5
    m1 = 1
    m2 = 0.1
    g = 9.8

    x, x_d, θ, θ_d = state

    state_new = np.array([x, x_d, θ-np.pi/2, θ_d])
    F_x = K@state_new*-1

    x_dd = (F_x - g*m2*sin(2*θ)/2 + l*m2*cos(θ)*θ_d**2)/(m1 + m2*cos(θ)**2)
    θ_dd = (F_x*sin(θ) - g*m1*cos(θ) - g*m2*cos(θ) + l*m2*sin(2*θ)*θ_d**2/2)/(l*(m1 + m2*cos(θ)**2))

    return [x_d, x_dd, θ_d, θ_dd]


#coneural
def cartpole_c(t, state):
    length = 0.5
    masscart = 1
    masspole = 0.1
    gravity = 9.8

    x, x_d, θ, θ_d = state
    
    force = K@state

    sintheta = sin(θ)
    costheta = cos(θ)
    total_mass = masscart + masspole
    polemass_length = masspole * length

    temp = (
        force + polemass_length * θ_d**2 * sintheta
    ) / total_mass
    thetaacc = (gravity * sintheta - costheta * temp) / (
        length * (4.0 / 3.0 - masspole * costheta**2 / total_mass)
    )
    xacc = temp - polemass_length * thetaacc * costheta / total_mass

    return [x_d, -xacc, θ_d, thetaacc]


deg_off = 1 #pole starts this many degrees to the side
t_bound = 20
max_step = 0.1
fig, axs = plt.subplots(2, 2, sharex='col', figsize=(10,8))

rk4 = RK45(cartpole, t0=0, y0=[0, 0, radians(90+deg_off), 0], t_bound=t_bound, max_step=max_step)
times = []
states = []
while rk4.t < t_bound:
    rk4.step()
    times.append(rk4.t)
    states.append(rk4.y)
    print(rk4.t, rk4.y)

times = np.array(times)
states = np.array(states)

axs[0, 0].plot(times, states[:,0], label='my own')
axs[0, 0].set_title('x')
axs[0, 1].plot(times, states[:,1])
axs[0, 1].set_title('x_dot')
axs[1, 0].plot(times, states[:,2]-radians(90))
axs[1, 0].set_title('θ')
axs[1, 1].plot(times, states[:,3])
axs[1, 1].set_title('θ_dot')

rk4 = RK45(cartpole_c, t0=0, y0=[0, 0, radians(deg_off), 0], t_bound=t_bound, max_step=max_step)
times = []
states = []
while rk4.t < t_bound:
    rk4.step()
    times.append(rk4.t)
    states.append(rk4.y)
    print(rk4.t, rk4.y)

times = np.array(times)
states = np.array(states)

axs[0, 0].plot(times, states[:,0], label='coneural')
axs[0, 0].legend()
axs[0, 0].set_title('x')
axs[0, 1].plot(times, states[:,1])
axs[0, 1].set_title('x_dot')
axs[1, 0].plot(times, states[:,2])
axs[1, 0].set_title('θ')
axs[1, 1].plot(times, states[:,3])
axs[1, 1].set_title('θ_dot')




plt.show()










