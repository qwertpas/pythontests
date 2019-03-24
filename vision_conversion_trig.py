
from math import *
a1 = 18
b1 = 90
s1 = -20  # measured as if looking at robot from the target, right is positive.

s1 = s1 * (pi/180)
h = atan2(a1, b1)
d = sqrt(a1*a1 + b1*b1)
a2 = d * sin(s1)
b2 = d * cos(s1)
s2 = (h + s1) * (180/pi)

print s2
