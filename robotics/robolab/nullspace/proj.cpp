#include <math.h>
#include <cstdio>

int main(int argc, char const *argv[]) {

    float th1_arm = 0.3;
    float th2_arm = 0.2;
    float th3_arm = 0.1;
    float th4_arm = 0.4;
    float th1_arm_des; float th2_arm_des; float th3_arm_des; float th4_arm_des; 
    float th_arm_des_vec[4];
    float th_arm_vec[4];
    float dth_arm[4];
    float def_R[6];
    float def_R_num[6];
    float F_arm[6];
    float tau_arm[8];

    float Jc_R[3][4];
    // Inverse jacobian & orthonormal basis variables
    float det_Jc[2];
    float det_Jc_nullcalc = 0.0001;
    float invdet = 0.0001; // Nonzero initialization
    float Jc_R_temp[3][3];
    float Jc_R_inner_inv[3][3];
    float Jc_R_inv[4][3];
    float tau_joint_des[4];
    float tau_null_space_task[4];
    float P_ortho[4][4];

    float Kp_arm[8];
    float Kd_arm[8];
    float Kp_arm_des[8];
    float Kd_arm_des[8];



    Jc_R[0][0] = 0.21144060406275340674442375643594 * sin(th4_arm) * (cos(th3_arm) * sin(th1_arm) - 1.0 * cos(th1_arm) * sin(th2_arm) * sin(th3_arm)) - 0.00414 * sin(th1_arm) - 0.11945 * cos(th1_arm) * cos(th2_arm) + 0.046167919236689778546001150516531 * cos(th4_arm) * (cos(th3_arm) * sin(th1_arm) - 1.0 * cos(th1_arm) * sin(th2_arm) * sin(th3_arm)) - 0.21144060406275340674442375643594 * cos(th1_arm) * cos(th2_arm) * cos(th4_arm) + 0.046167919236689778546001150516531 * cos(th1_arm) * cos(th2_arm) * sin(th4_arm);
    Jc_R[0][1] = 0.11945 * sin(th1_arm) * sin(th2_arm) + 0.21144060406275340674442375643594 * cos(th4_arm) * sin(th1_arm) * sin(th2_arm) - 0.046167919236689778546001150516531 * sin(th1_arm) * sin(th2_arm) * sin(th4_arm) - 0.046167919236689778546001150516531 * cos(th2_arm) * cos(th4_arm) * sin(th1_arm) * sin(th3_arm) - 0.21144060406275340674442375643594 * cos(th2_arm) * sin(th1_arm) * sin(th3_arm) * sin(th4_arm);
    Jc_R[0][2] = 0.21144060406275340674442375643594 * sin(th4_arm) * (cos(th1_arm) * sin(th3_arm) - 1.0 * cos(th3_arm) * sin(th1_arm) * sin(th2_arm)) + 0.046167919236689778546001150516531 * cos(th4_arm) * (cos(th1_arm) * sin(th3_arm) - 1.0 * cos(th3_arm) * sin(th1_arm) * sin(th2_arm));
    Jc_R[0][3] = 0.046167919236689778546001150516531 * sin(th4_arm) * (cos(th1_arm) * cos(th3_arm) + sin(th1_arm) * sin(th2_arm) * sin(th3_arm)) - 0.21144060406275340674442375643594 * cos(th4_arm) * (cos(th1_arm) * cos(th3_arm) + sin(th1_arm) * sin(th2_arm) * sin(th3_arm)) + 0.21144060406275340674442375643594 * cos(th2_arm) * sin(th1_arm) * sin(th4_arm) + 0.046167919236689778546001150516531 * cos(th2_arm) * cos(th4_arm) * sin(th1_arm);

    Jc_R[1][0] = 0;
    Jc_R[1][1] = 0.11945 * cos(th2_arm) + 0.21144060406275340674442375643594 * cos(th2_arm) * cos(th4_arm) - 0.046167919236689778546001150516531 * cos(th2_arm) * sin(th4_arm) + 0.046167919236689778546001150516531 * cos(th4_arm) * sin(th2_arm) * sin(th3_arm) + 0.21144060406275340674442375643594 * sin(th2_arm) * sin(th3_arm) * sin(th4_arm);
    Jc_R[1][2] = -0.046167919236689778546001150516531 * cos(th2_arm) * cos(th3_arm) * cos(th4_arm) - 0.21144060406275340674442375643594 * cos(th2_arm) * cos(th3_arm) * sin(th4_arm);
    Jc_R[1][3] = 0.046167919236689778546001150516531 * cos(th2_arm) * sin(th3_arm) * sin(th4_arm) - 0.21144060406275340674442375643594 * sin(th2_arm) * sin(th4_arm) - 0.046167919236689778546001150516531 * cos(th4_arm) * sin(th2_arm) - 0.21144060406275340674442375643594 * cos(th2_arm) * cos(th4_arm) * sin(th3_arm);

    Jc_R[2][0] = 0.11945 * cos(th2_arm) * sin(th1_arm) - 0.00414 * cos(th1_arm) + 0.046167919236689778546001150516531 * cos(th4_arm) * (cos(th1_arm) * cos(th3_arm) + sin(th1_arm) * sin(th2_arm) * sin(th3_arm)) + 0.21144060406275340674442375643594 * sin(th4_arm) * (cos(th1_arm) * cos(th3_arm) + sin(th1_arm) * sin(th2_arm) * sin(th3_arm)) - 0.046167919236689778546001150516531 * cos(th2_arm) * sin(th1_arm) * sin(th4_arm) + 0.21144060406275340674442375643594 * cos(th2_arm) * cos(th4_arm) * sin(th1_arm);
    Jc_R[2][1] = 0.11945 * cos(th1_arm) * sin(th2_arm) - 0.046167919236689778546001150516531 * cos(th1_arm) * sin(th2_arm) * sin(th4_arm) + 0.21144060406275340674442375643594 * cos(th1_arm) * cos(th4_arm) * sin(th2_arm) - 0.046167919236689778546001150516531 * cos(th1_arm) * cos(th2_arm) * cos(th4_arm) * sin(th3_arm) - 0.21144060406275340674442375643594 * cos(th1_arm) * cos(th2_arm) * sin(th3_arm) * sin(th4_arm);
    Jc_R[2][2] = -0.046167919236689778546001150516531 * cos(th4_arm) * (sin(th1_arm) * sin(th3_arm) + cos(th1_arm) * cos(th3_arm) * sin(th2_arm)) - 0.21144060406275340674442375643594 * sin(th4_arm) * (sin(th1_arm) * sin(th3_arm) + cos(th1_arm) * cos(th3_arm) * sin(th2_arm));
    Jc_R[2][3] = 0.21144060406275340674442375643594 * cos(th4_arm) * (cos(th3_arm) * sin(th1_arm) - 1.0 * cos(th1_arm) * sin(th2_arm) * sin(th3_arm)) - 0.046167919236689778546001150516531 * sin(th4_arm) * (cos(th3_arm) * sin(th1_arm) - 1.0 * cos(th1_arm) * sin(th2_arm) * sin(th3_arm)) + 0.046167919236689778546001150516531 * cos(th1_arm) * cos(th2_arm) * cos(th4_arm) + 0.21144060406275340674442375643594 * cos(th1_arm) * cos(th2_arm) * sin(th4_arm);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Jc_R_temp[i][j] = 0;
            for (int k = 0; k < 4; k++) {
                Jc_R_temp[i][j] += Jc_R[i][k] * Jc_R[j][k];
            }
        }
    }
    // Find inverse of inner jacobian product i.e Jc_R_temp
    det_Jc_nullcalc = Jc_R_temp[0][0] * (Jc_R_temp[1][1] * Jc_R_temp[2][2] - Jc_R_temp[2][1] * Jc_R_temp[1][2]) -
                      Jc_R_temp[0][1] * (Jc_R_temp[1][0] * Jc_R_temp[2][2] - Jc_R_temp[1][2] * Jc_R_temp[2][0]) +
                      Jc_R_temp[0][2] * (Jc_R_temp[1][0] * Jc_R_temp[2][1] - Jc_R_temp[1][1] * Jc_R_temp[2][0]);
    invdet = 1 / det_Jc_nullcalc;
    Jc_R_inner_inv[0][0] = (Jc_R_temp[1][1] * Jc_R_temp[2][2] - Jc_R_temp[2][1] * Jc_R_temp[1][2]) * invdet;
    Jc_R_inner_inv[0][1] = (Jc_R_temp[0][2] * Jc_R_temp[2][1] - Jc_R_temp[0][1] * Jc_R_temp[2][2]) * invdet;
    Jc_R_inner_inv[0][2] = (Jc_R_temp[0][1] * Jc_R_temp[1][2] - Jc_R_temp[0][2] * Jc_R_temp[1][1]) * invdet;
    Jc_R_inner_inv[1][0] = (Jc_R_temp[1][2] * Jc_R_temp[2][0] - Jc_R_temp[1][0] * Jc_R_temp[2][2]) * invdet;
    Jc_R_inner_inv[1][1] = (Jc_R_temp[0][0] * Jc_R_temp[2][2] - Jc_R_temp[0][2] * Jc_R_temp[2][0]) * invdet;
    Jc_R_inner_inv[1][2] = (Jc_R_temp[1][0] * Jc_R_temp[0][2] - Jc_R_temp[0][0] * Jc_R_temp[1][2]) * invdet;
    Jc_R_inner_inv[2][0] = (Jc_R_temp[1][0] * Jc_R_temp[2][1] - Jc_R_temp[2][0] * Jc_R_temp[1][1]) * invdet;
    Jc_R_inner_inv[2][1] = (Jc_R_temp[2][0] * Jc_R_temp[0][1] - Jc_R_temp[0][0] * Jc_R_temp[2][1]) * invdet;
    Jc_R_inner_inv[2][2] = (Jc_R_temp[0][0] * Jc_R_temp[1][1] - Jc_R_temp[1][0] * Jc_R_temp[0][1]) * invdet;

    // Final step: Jc_inv = Jc^T * Jc_inner_inv
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            Jc_R_inv[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                Jc_R_inv[i][j] += Jc_R[k][i] * Jc_R_inner_inv[j][k];
            }
        }
    }

    // Compute orthonormal basis for the kernel of Jacobian: P = (I - (J^+) * J)
    //  P_ortho initialized to be identity

    P_ortho[0][0] = 1; P_ortho[0][1] = 0; P_ortho[0][2] = 0; P_ortho[0][3] = 0;
    P_ortho[1][0] = 0; P_ortho[1][1] = 1; P_ortho[1][2] = 0; P_ortho[1][3] = 0;
    P_ortho[2][0] = 0; P_ortho[2][1] = 0; P_ortho[2][2] = 1; P_ortho[2][3] = 0;
    P_ortho[3][0] = 0; P_ortho[3][1] = 0; P_ortho[3][2] = 0; P_ortho[3][3] = 1;
    for (int i = 0; i < 4; i++) {
        tau_joint_des[i] = Kp_arm_des[i] * (th_arm_des_vec[i] - th_arm_vec[i]) + Kd_arm_des[i] * (0 - dth_arm[i]);
        tau_null_space_task[i] = 0;
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 3; k++) {
                P_ortho[i][j] -= Jc_R_inv[i][k] * Jc_R[k][j];
            }
            tau_null_space_task[i] += P_ortho[i][j] * tau_joint_des[j];
        }
    }

    printf("Jc_R_inner_inv:\n");
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            printf("%f ", Jc_R_inner_inv[i][j]);
        }
        printf("\n");
    }

    printf("Jc_R_inv:\n");
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            printf("%f ", Jc_R_inv[i][j]);
        }
        printf("\n");
    }

    printf("P_ortho matrix:\n");
    printf("%f %f %f %f\n", P_ortho[0][0], P_ortho[0][1], P_ortho[0][2], P_ortho[0][3]);
    printf("%f %f %f %f\n", P_ortho[1][0], P_ortho[1][1], P_ortho[1][2], P_ortho[1][3]);
    printf("%f %f %f %f\n", P_ortho[2][0], P_ortho[2][1], P_ortho[2][2], P_ortho[2][3]);
    printf("%f %f %f %f\n", P_ortho[3][0], P_ortho[3][1], P_ortho[3][2], P_ortho[3][3]);
}