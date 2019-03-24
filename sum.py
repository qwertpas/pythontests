from math import *
n = 0

lower = 1
upper = 1000

add = 1.2

for i in range(lower, upper + 1):

    # n=n+log(i+3, 10)
    n = n + add
    add = add / 2.0
    print n
