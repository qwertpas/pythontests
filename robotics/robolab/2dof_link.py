from sympy import symbols, cos, sin, Matrix, simplify

# Define symbolic variables
nJoints = 2
N, m, Ic, Lc, Ir, L, q, qd, qdd, tau = symbols('N m Ic Lc Ir L q qd qdd tau', cls=Matrix)
q = Matrix(q)
qd = Matrix(qd)
qdd = Matrix(qdd)
tau = Matrix(tau)

# Unit vectors (global coordinate system)
xhat = Matrix([1, 0])
yhat = Matrix([0, 1])

# Absolute joint angles
a1 = q[0]
a2 = q[0] + q[1]

# Unit vectors of limb directions
e1 = cos(a1) * xhat + sin(a1) * yhat
e2 = cos(a2) * xhat + sin(a2) * yhat

# Center of mass positions of links
G1 = Lc[0] * e1
G2 = L[0] * e1 + Lc[1] * e2

# A neat trick to get symbolic time-derivative
def derivative(x):
    return x.jacobian(q) * qd + x.jacobian(qd) * qdd

# Time-derivatives of center of mass positions
dG1 = derivative(G1)
dG2 = derivative(G2)

# Time-derivatives of absolute joint angles
da1 = derivative(a1)
da2 = derivative(a2)

# Kinetic energy of structural parts
T1 = 0.5 * (m[0] * dG1.dot(dG1) + m[1] * dG2.dot(dG2))  # rectilinear
T2 = 0.5 * (Ic[0] * da1.dot(da1) + Ic[1] * da2.dot(da2))  # rotational

# Kinetic energy of actuators
dr1 = N[0] * qd[0]  # rotors run x(gearRatio) times faster than joints
dr2 = N[1] * qd[1]
T3 = 0.5 * (Ir[0] * dr1 * dr1 + Ir[1] * dr2 * dr2)

# Define Lagrangian
T = T1 + T2 + T3  # Kinetic Energy
V = 0  # Potential Energy
Lag = T - V  # Lagrangian

# Define Generalized Work
dW = tau.dot(qd)

# Euler-Lagrangian Equation of Motion (EoM)
EoM_LHS = Matrix.zeros(nJoints, 1)
Lag = simplify(Lag)
for jj in range(nJoints):
    partial_dq_ = Lag.diff(qd[jj])
    partial_q_ = Lag.diff(q[jj])
    EoM_LHS[jj, 0] = derivative(partial_dq_) - partial_q_
EoM_RHS = (dW.jacobian(qd)).T

# Decompose EoM
M = EoM_LHS.jacobian(qdd)  # hessian(T, qd) also works
c = EoM_LHS - M * qdd