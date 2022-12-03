import numpy as np
from numpy.random import uniform
from numpy import sqrt, pi, sin, cos
from numpy.linalg import norm
from scipy.spatial import KDTree
import matplotlib.pyplot as plt

N = 128 #number of nodes in PRM graph
k = 5  #number of closest neighbors to check

obstacles = np.array([
    [[-2, 0], 1],
    [[2, 2], sqrt(2)],
    [[1, -2], sqrt(5)]
], dtype='object')

def circle_collision(p1, p2, c, r):
    '''check if a line segment from p1 to p2 collides with a circle with center c and radius r'''
    (p1x, p1y), (p2x, p2y), (cx, cy) = p1, p2, c
    (x1, y1), (x2, y2) = (p1x - cx, p1y - cy), (p2x - cx, p2y - cy)
    dx, dy = (x2 - x1), (y2 - y1)
    dr = (dx ** 2 + dy ** 2)**.5
    big_d = x1 * y2 - x2 * y1
    discriminant = r ** 2 * dr ** 2 - big_d ** 2

    if discriminant < 0:  # No intersection between circle and line
        return []
    else:  # There may be 0, 1, or 2 intersections with the segment
        intersections = [
            (cx + (big_d * dy + sign * (-1 if dy < 0 else 1) * dx * discriminant**.5) / dr ** 2,
             cy + (-big_d * dx + sign * abs(dy) * discriminant**.5) / dr ** 2)
            for sign in ((1, -1) if dy < 0 else (-1, 1))]  # This makes sure the order along the segment is correct

        fraction_along_segment = [(xi - p1x) / dx if abs(dx) > abs(dy) else (yi - p1y) / dy for xi, yi in intersections]
        intersections = [pt for pt, frac in zip(intersections, fraction_along_segment) if 0 <= frac <= 1]

    return (len(intersections) > 0)


nodes = np.zeros((128,2)) #128 rows of (x, y) points
for i in range(128):
    invalid = True
    #uniformly sample within the large circle, reject if inside an obstacle
    while invalid:
        r = sqrt(uniform(0, 25))
        theta = uniform(0, 2*pi)
        pt = np.array([r*cos(theta), r*sin(theta)])

        invalid = False
        for circle in obstacles:
            if norm(pt - circle[0]) < circle[1]:
                invalid = True

    nodes[i] = pt

#generate KD tree for easy neighbor search
tree = KDTree(nodes)

#list of (node1 index, node2 index) representing connections between nodes. Order doesn't matter
edges = []
edges_c = []

for node_i in range(len(nodes)):
    neighbor_dists, neighbor_is = tree.query(nodes[node_i], k+1) #closest "neighbor" is the node itself, so search 1 more

    for neighbor_i in neighbor_is[1:]:

        edge = sorted((node_i, neighbor_i))

        # #check if this combination of node indexes is already an edge
        already_exists = False
        for prev_edge in edges:
            if np.all(edge == prev_edge):
                already_exists = True
                break
        if already_exists:
            continue

        #check if neighbor is visible from node
        collision = False
        for circle in obstacles:
            if circle_collision(nodes[node_i], nodes[neighbor_i], circle[0], circle[1]):
                collision = True
                break
        if not collision:
            edges.append(edge)
        else:
            edges_c.append(edge)
         
print(sorted(edges))

''' PLOTTING '''
fig, ax = plt.subplots(figsize=(10,8)) # note we must use plt.subplots, not plt.subplot
ax.set_aspect('equal')
ax.set_xlim(-5, 5)
ax.set_ylim(-5, 5)
ax.set_title(f"PRM with {N} nodes, nearest {k} neighbors")

circle1 = plt.Circle((-2, 0), 1, color='r')
circle2 = plt.Circle((2, 2), sqrt(2), color='blue')
circle3 = plt.Circle((1, -2), sqrt(5), color='green')
boundary = plt.Circle((0,0), 5, fill=False, color='black')
ax.add_patch(circle1)
ax.add_patch(circle2)
ax.add_patch(circle3)
ax.add_patch(boundary)



for edge in edges:
    p1 = nodes[edge[0]]
    p2 = nodes[edge[1]]
    ax.plot((p1[0], p2[0]), (p1[1], p2[1]), c='gray', linewidth=1)

#uncomment to draw the edges that do collide
# for edge in edges_c:
#     p1 = nodes[edge[0]]
#     p2 = nodes[edge[1]]
#     ax.plot((p1[0], p2[0]), (p1[1], p2[1]), c='yellow', linewidth=1)

for node in nodes:
    ax.scatter(x=nodes[:,0], y=nodes[:,1], c='orange')

plt.show()
