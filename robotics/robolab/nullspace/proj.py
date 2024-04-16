from math import sin, cos
import numpy as np

th1_arm = 0.3
th2_arm = 0.2
th3_arm = 0.1
th4_arm = 0.4

Jc_R = np.zeros((3,4))
Jc_R[0][0] = 0.21144060406275340674442375643594*sin(th4_arm)*(cos(th3_arm)*sin(th1_arm) - 1.0*cos(th1_arm)*sin(th2_arm)*sin(th3_arm)) - 0.00414*sin(th1_arm) - 0.11945*cos(th1_arm)*cos(th2_arm) + 0.046167919236689778546001150516531*cos(th4_arm)*(cos(th3_arm)*sin(th1_arm) - 1.0*cos(th1_arm)*sin(th2_arm)*sin(th3_arm)) - 0.21144060406275340674442375643594*cos(th1_arm)*cos(th2_arm)*cos(th4_arm) + 0.046167919236689778546001150516531*cos(th1_arm)*cos(th2_arm)*sin(th4_arm);
Jc_R[0][1] = 0.11945*sin(th1_arm)*sin(th2_arm) + 0.21144060406275340674442375643594*cos(th4_arm)*sin(th1_arm)*sin(th2_arm) - 0.046167919236689778546001150516531*sin(th1_arm)*sin(th2_arm)*sin(th4_arm) - 0.046167919236689778546001150516531*cos(th2_arm)*cos(th4_arm)*sin(th1_arm)*sin(th3_arm) - 0.21144060406275340674442375643594*cos(th2_arm)*sin(th1_arm)*sin(th3_arm)*sin(th4_arm);
Jc_R[0][2] = 0.21144060406275340674442375643594*sin(th4_arm)*(cos(th1_arm)*sin(th3_arm) - 1.0*cos(th3_arm)*sin(th1_arm)*sin(th2_arm)) + 0.046167919236689778546001150516531*cos(th4_arm)*(cos(th1_arm)*sin(th3_arm) - 1.0*cos(th3_arm)*sin(th1_arm)*sin(th2_arm));
Jc_R[0][3] = 0.046167919236689778546001150516531*sin(th4_arm)*(cos(th1_arm)*cos(th3_arm) + sin(th1_arm)*sin(th2_arm)*sin(th3_arm)) - 0.21144060406275340674442375643594*cos(th4_arm)*(cos(th1_arm)*cos(th3_arm) + sin(th1_arm)*sin(th2_arm)*sin(th3_arm)) + 0.21144060406275340674442375643594*cos(th2_arm)*sin(th1_arm)*sin(th4_arm) + 0.046167919236689778546001150516531*cos(th2_arm)*cos(th4_arm)*sin(th1_arm);

Jc_R[1][0] = 0;
Jc_R[1][1] = 0.11945*cos(th2_arm) + 0.21144060406275340674442375643594*cos(th2_arm)*cos(th4_arm) - 0.046167919236689778546001150516531*cos(th2_arm)*sin(th4_arm) + 0.046167919236689778546001150516531*cos(th4_arm)*sin(th2_arm)*sin(th3_arm) + 0.21144060406275340674442375643594*sin(th2_arm)*sin(th3_arm)*sin(th4_arm);
Jc_R[1][2] = -0.046167919236689778546001150516531*cos(th2_arm)*cos(th3_arm)*cos(th4_arm) - 0.21144060406275340674442375643594*cos(th2_arm)*cos(th3_arm)*sin(th4_arm);
Jc_R[1][3] = 0.046167919236689778546001150516531*cos(th2_arm)*sin(th3_arm)*sin(th4_arm) - 0.21144060406275340674442375643594*sin(th2_arm)*sin(th4_arm) - 0.046167919236689778546001150516531*cos(th4_arm)*sin(th2_arm) - 0.21144060406275340674442375643594*cos(th2_arm)*cos(th4_arm)*sin(th3_arm);

Jc_R[2][0] = 0.11945*cos(th2_arm)*sin(th1_arm) - 0.00414*cos(th1_arm) + 0.046167919236689778546001150516531*cos(th4_arm)*(cos(th1_arm)*cos(th3_arm) + sin(th1_arm)*sin(th2_arm)*sin(th3_arm)) + 0.21144060406275340674442375643594*sin(th4_arm)*(cos(th1_arm)*cos(th3_arm) + sin(th1_arm)*sin(th2_arm)*sin(th3_arm)) - 0.046167919236689778546001150516531*cos(th2_arm)*sin(th1_arm)*sin(th4_arm) + 0.21144060406275340674442375643594*cos(th2_arm)*cos(th4_arm)*sin(th1_arm);
Jc_R[2][1] = 0.11945*cos(th1_arm)*sin(th2_arm) - 0.046167919236689778546001150516531*cos(th1_arm)*sin(th2_arm)*sin(th4_arm) + 0.21144060406275340674442375643594*cos(th1_arm)*cos(th4_arm)*sin(th2_arm) - 0.046167919236689778546001150516531*cos(th1_arm)*cos(th2_arm)*cos(th4_arm)*sin(th3_arm) - 0.21144060406275340674442375643594*cos(th1_arm)*cos(th2_arm)*sin(th3_arm)*sin(th4_arm);
Jc_R[2][2] = -0.046167919236689778546001150516531*cos(th4_arm)*(sin(th1_arm)*sin(th3_arm) + cos(th1_arm)*cos(th3_arm)*sin(th2_arm)) - 0.21144060406275340674442375643594*sin(th4_arm)*(sin(th1_arm)*sin(th3_arm) + cos(th1_arm)*cos(th3_arm)*sin(th2_arm));
Jc_R[2][3] = 0.21144060406275340674442375643594*cos(th4_arm)*(cos(th3_arm)*sin(th1_arm) - 1.0*cos(th1_arm)*sin(th2_arm)*sin(th3_arm)) - 0.046167919236689778546001150516531*sin(th4_arm)*(cos(th3_arm)*sin(th1_arm) - 1.0*cos(th1_arm)*sin(th2_arm)*sin(th3_arm)) + 0.046167919236689778546001150516531*cos(th1_arm)*cos(th2_arm)*cos(th4_arm) + 0.21144060406275340674442375643594*cos(th1_arm)*cos(th2_arm)*sin(th4_arm);

P = np.eye(4,4) - np.linalg.pinv(Jc_R)@Jc_R

tau_task2 = np.array([1,2,3,4])

# print("Jc_R\n", Jc_R)
pinv_right = Jc_R.T @ np.linalg.inv(Jc_R@Jc_R.T)

# print("Jc_R\n", Jc_R)
print("Jc_R_inner_inv\n", np.linalg.inv(Jc_R@Jc_R.T))
# print("pinv\n", np.linalg.pinv(Jc_R))
print("pinv_right\n", pinv_right)
print("P\n", P)
print("projectedtau2\n", P@tau_task2)
print("should be 0\n", Jc_R@(P@tau_task2))