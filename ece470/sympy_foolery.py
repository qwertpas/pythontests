from sympy import *
import numpy as np

V0, d, w, z, t, ω = symbols('V0 d w z t ω')

Ex = cos(pi*z+pi/2)*cos(ω*t)
Hy = sin(pi*z+pi/2)*sin(ω*t)
print(simplify(integrate(Ex**2, z)))
print(simplify(integrate(Hy**2, z)))