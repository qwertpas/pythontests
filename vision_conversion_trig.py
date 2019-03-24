'''convert_trig function and using it to get vision data'''

from math import pi, atan2, sin, cos, sqrt



def convert_trig(x_to_target, y_to_target, skew):
    '''convert measurements relative to robot into measurements relative to target perpendicular'''
    a1 = x_to_target  # x dist to target relative to robot
    b1 = y_to_target  # y dist to target relative to robot
    s1 = skew  # measured as if looking at robot from the target, left is positive.

    s1 = s1 * (pi/180.0)  # converting to radians for trig
    h = atan2(a1, b1)  # horizontal angle in radians
    d = sqrt(a1*a1 + b1*b1)  # pythagorean theorem for distance

    # with a 2 after it means it is measurement to robot relative from target
    a2 = d * sin(s1)
    b2 = d * cos(s1)
    s2 = (h + s1) * (180.0/pi)  # skew in degrees (directly facing target is 0, to the right is positive)

    return a2, b2, s2 # new x, y, skew

print convert_trig(18, 90, -20)
