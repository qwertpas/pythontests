import control as ctrl
import matplotlib

# Define transfer functions
G = ctrl.tf([1], [1, 1, 0])         # G(s) = 1 / (s(s+1))
C = ctrl.tf([10, 10], [1])          # C(s) = 10(s + 1)

# Open-loop and closed-loop
L = ctrl.series(C, G)
T = ctrl.feedback(L)

# Analyze phase margin and bandwidth
ctrl.bode(L)
