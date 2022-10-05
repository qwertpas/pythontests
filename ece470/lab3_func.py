#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""


def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix

	l1 = np.array([-150,150, 162]) / 1000.
	l2 = np.array([0, 120, 0]) / 1000.
	l3 = np.array([244, 0, 0]) / 1000.
	l4 = np.array([213,-93, 0]) / 1000.
	l5 = np.array([0, 83, 0]) / 1000.
	l6 = np.array([83, 0, 0]) / 1000.



	w1 = np.array([0,0,1])
	q1 = l1
	v1 = np.cross(-w1, q1)

	w2 = np.array ([0,1,0])
	q2 = q1 + l2
	v2 = np.cross(-w2, q2)

	w3 = np.array ([0,1,0])
	q3 = q2 + l3
	v3 = np.cross(-w3, q3)

	w4 = np.array ([0,1,0])
	q4 = q3 + l4
	v4 = np.cross(-w4, q4)

	w5 = np.array ([1,0,0])
	q5 = q4 + l5

	v5 = np.cross(-w5, q5)

	w6 = np.array ([0,1,0])
	q6 = q5 + l6
	v6 = np.cross(-w6, q6)
	print(f"q6: {q6}")


	# w1 = np.array([0,0,1])
	# q1 = np.array([-150,150,10]) / 1000.
	# v1 = np.cross(-w1, q1)

	# w2 = np.array ([0,1,0])
	# q2 = np.array([0, 120, 0]) / 1000.
	# v2 = np.cross(-w2, q2)

	# w3 = np.array ([0,1,0])
	# q3 = np.array([244, 0, 0]) / 1000.
	# v3 = np.cross(-w3, q3)

	# w4 = np.array ([0,1,0])
	# q4 = np.array([213,-93, 0]) / 1000.
	# v4 = np.cross(-w4, q4)

	# w5 = np.array ([1,0,0])
	# q5 = np.array([0, 83, 0]) / 1000.
	# v5 = np.cross(-w5, q5)

	# w6 = np.array ([0,1,0])
	# q6 = np.array([0, 83, 0]) / 1000.
	# v6 = np.cross(-w6, q6)

	

	W = np.array([w1, w2, w3, w4, w5, w6])
	V = np.array([v1, v2, v3, v4, v5, v6])

	S = np.zeros((6, 4, 4))
	for i in range(6):
		S[i] = np.array([
			[0, -W[i][2], W[i][1], V[i][0]],
			[W[i][2], 0, -W[i][0], V[i][1]],
			[-W[i][1], W[i][0], 0, V[i][2]],
			[0, 0, 0, 0]
		])

	Mx = 40 / 1000.
	My = 41 / 1000.
	Mz = 22 / 1000.

	# Mx = 0 / 1000.
	# My = 150 / 1000.
	# Mz = 53.5 / 1000.

	M = np.array([
		[0, -1, 0, Mx],
		[0, 0, -1, My],
		[1, 0, 0, Mz],
		[0, 0, 0, 1]
	])

	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#

	M, S = Get_MS()

	print(f"M: {str(M)} \n")
	
	print(f"S: {str(S)} \n")

	T = expm(S[0]*(theta1)) @ expm(S[1]*theta2) @ expm(S[2]*(theta3)) @ expm(S[3]*theta4) @ expm(S[4]*theta5) @ expm(S[5]*theta6)


	# ==============================================================#

	print(f"T: {str(T)}\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value
