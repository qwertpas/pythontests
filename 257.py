import numpy as np

# import numpy

# def gs_cofficient(v1, v2):
#     return numpy.dot(v2, v1) / numpy.dot(v1, v1)

# def multiply(cofficient, v):
#     return map((lambda x : x * cofficient), v)

# def proj(v1, v2):
#     return multiply(gs_cofficient(v1, v2) , v1)

# def gs(X):
#     Y = []
#     for i in range(len(X)):
#         temp_vec = X[i]
#         for inY in Y :
#             proj_vec = proj(inY, X[i])
#             #print "i =", i, ", projection vector =", proj_vec
#             temp_vec = map(lambda x, y : x - y, temp_vec, proj_vec)
#             #print "i =", i, ", temporary vector =", temp_vec
#         Y.append(temp_vec)
#     return Y

# test2 = numpy.array([
#     [1.0, 1.0, 0.0], 
#     [1.0, 3.0, 1.0], 
#     [2.0, -1.0, 1.0]
# ])

# print(numpy.array(gs(test2)))


A = np.array([
    [7, 1, 1],
    [1, 7, 1],
    [1, 1, 7]
])

# A_inv = np.array([
#     [2, 4, -2],
#     [-4, 10, 4],
#     [4, -1, 5]
# ])

print(np.linalg.eig(A)[0])
print(np.linalg.eig(A)[1])

# print(A @ A_inv)


