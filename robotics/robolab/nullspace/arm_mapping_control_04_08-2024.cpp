// *****************************        ARM CONTROL MODES         ***************************** //
//Parameters
int sensor_hand = 0;
int POS_CTRL = 0;
int FORCE_CTRL = 1;
int ARM_MODE = 0;
int contactState = 0;
float th1_arm; float th2_arm;
float th3_arm; float th4_arm;  
float th1_arm_des; float th2_arm_des; float th3_arm_des; float th4_arm_des; 
float th_arm_des_vec[4];
float th_arm_vec[4];
float dth_arm[4];
float def_R[6];
float def_R_num[6];
float F_arm[6];
float tau_arm[8];
float Jc_R[3][4];
//Moment calculation vars
float r_i[6]; // vector from CoM to ef
float M_ext[6]; // estimated external moment
//Switching gains/modes params
float t_switch = 70.0; // ms switch time 
float t_duration = (t_us / 1000) - t_start_arm;
float Kp_arm[8];
float Kd_arm[8];
float Kp_arm_des[8];
float Kd_arm_des[8];

//Inverse jacobian & orthonormal basis variables
float det_Jc[2]; 
float det_Jc_nullcalc = 0.0001;
float invdet = 0.0001;// Nonzero initialization
float Jc_R_temp[3][3];
float Jc_R_inner_inv[3][3];
float Jc_R_inv[4][3];
float tau_joint_des[4];
float tau_null_space_task[4];
float P_ortho[4][4];

//Zeroing & initialization 
def_R[0] = 0; def_R[1] = 0; def_R[2] = 0;
tau_arm[0] = 0.0; tau_arm[1] = 0.0; tau_arm[2] = 0.0; tau_arm[3] = 0.0;
tau_arm[4] = 0.0; tau_arm[5] = 0.0; tau_arm[6] = 0.0; tau_arm[7] = 0.0;
//Initialize P_ortho to identity is important
P_ortho[0][0] = 1; P_ortho[0][1] = 0; P_ortho[0][2] = 0; P_ortho[0][3] = 0;
P_ortho[1][0] = 0; P_ortho[1][1] = 1; P_ortho[1][2] = 0; P_ortho[1][3] = 0;
P_ortho[2][0] = 0; P_ortho[2][1] = 0; P_ortho[2][2] = 1; P_ortho[2][3] = 0;
P_ortho[3][0] = 0; P_ortho[3][1] = 0; P_ortho[3][2] = 0; P_ortho[3][3] = 1;
for(i = 0; i < 8; i++){Kp_arm[i] = 0; Kd_arm[i] = 0;}


//Onboard motor PD controller gains
//Right arm 
Kp_arm_des[0] = 60; Kp_arm_des[1] = 40; Kp_arm_des[2] = 20; Kp_arm_des[3] = 40; 
Kd_arm_des[0] = 4.0; Kd_arm_des[1] = 4.0; Kd_arm_des[2] = 2.0; Kd_arm_des[3] = 2.0; 
//Left arm
Kp_arm_des[4] = 60; Kp_arm_des[5] = 40; Kp_arm_des[6] = 20; Kp_arm_des[7] = 40; 
Kd_arm_des[4] = 4.0; Kd_arm_des[5] = 4.0; Kd_arm_des[6] = 2.0; Kd_arm_des[7] = 2.0; 

// Spring-damper parameters
// float Kx_spring = forceGains[0];
// float Ky_spring = forceGains[1];
// float Kz_spring = forceGains[2];
// float  Kdx_spring = forceGains[3];
// float  Kdy_spring = forceGains[4];
// float  Kdz_spring = forceGains[5];

float Kx_spring = 200.0;
float Ky_spring = 200.0;
float Kz_spring = 200.0;
float  Kdx_spring = 2.35;
float  Kdy_spring = 2.35;
float  Kdz_spring = 2.35;


if(homePositionArm == 1){
    arm_home_pos[0] = R_motor_pos[0];
    arm_home_pos[1] = R_motor_pos[1];
    arm_home_pos[2] = R_motor_pos[2];
    arm_home_pos[3] = R_motor_pos[3];
    arm_home_pos[4] = L_motor_pos[0];
    arm_home_pos[5] = L_motor_pos[1];
    arm_home_pos[6] = L_motor_pos[2];
    arm_home_pos[7] = L_motor_pos[3];

    //Forward kinematics of ef home positon with force sensor
    //Cartesian space  home arm position WITHOUT force sensor length
    ef_R_home[0] = 0.00414*cos(th1_ra) - 0.11945*cos(th2_ra)*sin(th1_ra) - 0.046167919236689778546001150516531*cos(th4_ra)*(cos(th1_ra)*cos(th3_ra) + sin(th1_ra)*sin(th2_ra)*sin(th3_ra)) - 0.21144060406275340674442375643594*sin(th4_ra)*(cos(th1_ra)*cos(th3_ra) + sin(th1_ra)*sin(th2_ra)*sin(th3_ra)) + 0.046167919236689778546001150516531*cos(th2_ra)*sin(th1_ra)*sin(th4_ra) - 0.21144060406275340674442375643594*cos(th2_ra)*cos(th4_ra)*sin(th1_ra);
    ef_R_home[1] = 0.11945*sin(th2_ra) + 0.21144060406275340674442375643594*cos(th4_ra)*sin(th2_ra) - 0.046167919236689778546001150516531*sin(th2_ra)*sin(th4_ra) - 0.21144060406275340674442375643594*cos(th2_ra)*sin(th3_ra)*sin(th4_ra) - 0.046167919236689778546001150516531*cos(th2_ra)*cos(th4_ra)*sin(th3_ra) - 0.17668;
    ef_R_home[2] = 0.21144060406275340674442375643594*sin(th4_ra)*(cos(th3_ra)*sin(th1_ra) - 1.0*cos(th1_ra)*sin(th2_ra)*sin(th3_ra)) - 0.00414*sin(th1_ra) - 0.11945*cos(th1_ra)*cos(th2_ra) + 0.046167919236689778546001150516531*cos(th4_ra)*(cos(th3_ra)*sin(th1_ra) - 1.0*cos(th1_ra)*sin(th2_ra)*sin(th3_ra)) - 0.21144060406275340674442375643594*cos(th1_ra)*cos(th2_ra)*cos(th4_ra) + 0.046167919236689778546001150516531*cos(th1_ra)*cos(th2_ra)*sin(th4_ra);

    ef_R_home[3] = 0.00414*cos(th1_la) - 0.11945*cos(th2_la)*sin(th1_la) - 0.046167919236689778546001150516531*cos(th4_la)*(cos(th1_la)*cos(th3_la) + sin(th1_la)*sin(th2_la)*sin(th3_la)) - 0.21144060406275340674442375643594*sin(th4_la)*(cos(th1_la)*cos(th3_la) + sin(th1_la)*sin(th2_la)*sin(th3_la)) + 0.046167919236689778546001150516531*cos(th2_la)*sin(th1_la)*sin(th4_la) - 0.21144060406275340674442375643594*cos(th2_la)*cos(th4_la)*sin(th1_la);
    ef_R_home[4] = 0.11945*sin(th2_la) + 0.21144060406275340674442375643594*cos(th4_la)*sin(th2_la) - 0.046167919236689778546001150516531*sin(th2_la)*sin(th4_la) - 0.21144060406275340674442375643594*cos(th2_la)*sin(th3_la)*sin(th4_la) - 0.046167919236689778546001150516531*cos(th2_la)*cos(th4_la)*sin(th3_la) - 0.17668;
    ef_R_home[5] = 0.21144060406275340674442375643594*sin(th4_la)*(cos(th3_la)*sin(th1_la) - 1.0*cos(th1_la)*sin(th2_la)*sin(th3_la)) - 0.00414*sin(th1_la) - 0.11945*cos(th1_la)*cos(th2_la) + 0.046167919236689778546001150516531*cos(th4_la)*(cos(th3_la)*sin(th1_la) - 1.0*cos(th1_la)*sin(th2_la)*sin(th3_la)) - 0.21144060406275340674442375643594*cos(th1_la)*cos(th2_la)*cos(th4_la) + 0.046167919236689778546001150516531*cos(th1_la)*cos(th2_la)*sin(th4_la);

    //Cartesian space  home arm position WITH force sensor length
    //  ef_R_home[0] = 0.00414*cos(th1_ra) - 0.11945*cos(th2_ra)*sin(th1_ra) - 0.044307643397825228266384556263802*cos(th4_ra)*(cos(th1_ra)*cos(th3_ra) + sin(th1_ra)*sin(th2_ra)*sin(th3_ra)) - 0.28991855876216779175358340125968*sin(th4_ra)*(cos(th1_ra)*cos(th3_ra) + sin(th1_ra)*sin(th2_ra)*sin(th3_ra)) + 0.044307643397825228266384556263802*cos(th2_ra)*sin(th1_ra)*sin(th4_ra) - 0.28991855876216779175358340125968*cos(th2_ra)*cos(th4_ra)*sin(th1_ra);
    //  ef_R_home[1] = 0.11945*sin(th2_ra) + 0.28991855876216779175358340125968*cos(th4_ra)*sin(th2_ra) - 0.044307643397825228266384556263802*sin(th2_ra)*sin(th4_ra) - 0.28991855876216779175358340125968*cos(th2_ra)*sin(th3_ra)*sin(th4_ra) - 0.044307643397825228266384556263802*cos(th2_ra)*cos(th4_ra)*sin(th3_ra) - 0.17668;
    //  ef_R_home[2] = 0.28991855876216779175358340125968*sin(th4_ra)*(cos(th3_ra)*sin(th1_ra) - 1.0*cos(th1_ra)*sin(th2_ra)*sin(th3_ra)) - 0.00414*sin(th1_ra) - 0.11945*cos(th1_ra)*cos(th2_ra) + 0.044307643397825228266384556263802*cos(th4_ra)*(cos(th3_ra)*sin(th1_ra) - 1.0*cos(th1_ra)*sin(th2_ra)*sin(th3_ra)) - 0.28991855876216779175358340125968*cos(th1_ra)*cos(th2_ra)*cos(th4_ra) + 0.044307643397825228266384556263802*cos(th1_ra)*cos(th2_ra)*sin(th4_ra);

    //  ef_R_home[3] = 0.00414*cos(th1_la) - 0.11945*cos(th2_la)*sin(th1_la) - 0.044307643397825228266384556263802*cos(th4_la)*(cos(th1_la)*cos(th3_la) + sin(th1_la)*sin(th2_la)*sin(th3_la)) - 0.28991855876216779175358340125968*sin(th4_la)*(cos(th1_la)*cos(th3_la) + sin(th1_la)*sin(th2_la)*sin(th3_la)) + 0.044307643397825228266384556263802*cos(th2_la)*sin(th1_la)*sin(th4_la) - 0.28991855876216779175358340125968*cos(th2_la)*cos(th4_la)*sin(th1_la);
    //  ef_R_home[4] = 0.11945*sin(th2_la) + 0.28991855876216779175358340125968*cos(th4_la)*sin(th2_la) - 0.044307643397825228266384556263802*sin(th2_la)*sin(th4_la) - 0.28991855876216779175358340125968*cos(th2_la)*sin(th3_la)*sin(th4_la) - 0.044307643397825228266384556263802*cos(th2_la)*cos(th4_la)*sin(th3_la) - 0.17668;
    //  ef_R_home[5] = 0.28991855876216779175358340125968*sin(th4_la)*(cos(th3_la)*sin(th1_la) - 1.0*cos(th1_la)*sin(th2_la)*sin(th3_la)) - 0.00414*sin(th1_la) - 0.11945*cos(th1_la)*cos(th2_la) + 0.044307643397825228266384556263802*cos(th4_la)*(cos(th3_la)*sin(th1_la) - 1.0*cos(th1_la)*sin(th2_la)*sin(th3_la)) - 0.28991855876216779175358340125968*cos(th1_la)*cos(th2_la)*cos(th4_la) + 0.044307643397825228266384556263802*cos(th1_la)*cos(th2_la)*sin(th4_la);
}

//*********** START ARM CONTROL *************//

//SWITCHING ARM MODES FUNCTION
// if(user_trigger_L == 0){
//     ARM_MODE = POS_CTRL;
// }
// else if(user_trigger_L == 1){
//     ARM_MODE = FORCE_CTRL;
// }

if(startArmForceCtrl == 0){
    ARM_MODE = POS_CTRL;
}
else if(startArmForceCtrl == 1){
    ARM_MODE = FORCE_CTRL;
}

// CONTACT STATE (for sending to CRIO)
if(ARM_MODE == POS_CTRL){
    if((contactMade[0] > 2.5) && (contactMade[1] > 2.5)){
        contactState = 0; // Neither hand in contact
    }
    else if((contactMade[0] < 2.5) && (contactMade[1] > 2.5)){
        contactState = 1; // Only right hand in contact
    }
    else if((contactMade[0] > 2.5) && (contactMade[1] < 2.5)){
        contactState = 2; // Only left hand in contact
    }
    else if((contactMade[0] < 2.5) && (contactMade[1] < 2.5)){
        contactState = 3; // Both hands in contact
    }
    else{
        contactState = 0; // Anamoly edge case
    }
}
else{
    contactState = -1; // Arm in force control mode
}

//FORCE IMP CONTROL CALCULATIONS
for(sensor_hand = 0; sensor_hand < 2; ++sensor_hand){
    if(sensor_hand == 0){
        th1_arm = th1_ra; th2_arm = th2_ra;
        th3_arm = th3_ra; th4_arm = th4_ra;

        th1_arm_des = -th_R_Arm[0]; th2_arm_des =  th_R_Arm[1];
        th3_arm_des =  th_R_Arm[2]; th4_arm_des = -th_R_Arm[3];

        dth_arm[0] = dth1_ra; dth_arm[1] = dth2_ra;
        dth_arm[2] = dth3_ra; dth_arm[3] = dth4_ra;
    }
    else{
        th1_arm = th1_la; th2_arm = th2_la;
        th3_arm = th3_la; th4_arm = th4_la;

        th1_arm_des =  th_L_Arm[0]; th2_arm_des = -th_L_Arm[1];
        th3_arm_des = -th_L_Arm[2]; th4_arm_des =  th_L_Arm[3];

        dth_arm[0] = dth1_la; dth_arm[1] = dth2_la;
        dth_arm[2] = dth3_la; dth_arm[3] = dth4_la;
    }
    // Elbow singularity avoidance
    if((ARM_MODE == FORCE_CTRL) && (abs(th4_arm_des) < 0.05)){
        th4_arm_des = 0.05; // a negative desired elbow joint angle should not be possible 
    }

    // Desired vector construction
    th_arm_des_vec[0] = th1_arm_des; th_arm_des_vec[1] = th2_arm_des; 
    th_arm_des_vec[2] = th3_arm_des; th_arm_des_vec[3] = th4_arm_des;
    //Actual angle construction
    th_arm_vec[0] = th1_arm; th_arm_vec[1] = th2_arm;
    th_arm_vec[2] = th3_arm; th_arm_vec[3] = th4_arm;

    //Actual cartesian space arm position WITHOUT force sensor length
    ef_R[3*sensor_hand + 0] = 0.00414*cos(th1_arm) - 0.11945*cos(th2_arm)*sin(th1_arm) - 0.046167919236689778546001150516531*cos(th4_arm)*(cos(th1_arm)*cos(th3_arm) + sin(th1_arm)*sin(th2_arm)*sin(th3_arm)) - 0.21144060406275340674442375643594*sin(th4_arm)*(cos(th1_arm)*cos(th3_arm) + sin(th1_arm)*sin(th2_arm)*sin(th3_arm)) + 0.046167919236689778546001150516531*cos(th2_arm)*sin(th1_arm)*sin(th4_arm) - 0.21144060406275340674442375643594*cos(th2_arm)*cos(th4_arm)*sin(th1_arm);
    ef_R[3*sensor_hand + 1] = 0.11945*sin(th2_arm) + 0.21144060406275340674442375643594*cos(th4_arm)*sin(th2_arm) - 0.046167919236689778546001150516531*sin(th2_arm)*sin(th4_arm) - 0.21144060406275340674442375643594*cos(th2_arm)*sin(th3_arm)*sin(th4_arm) - 0.046167919236689778546001150516531*cos(th2_arm)*cos(th4_arm)*sin(th3_arm) - 0.17668;
    ef_R[3*sensor_hand + 2] = 0.21144060406275340674442375643594*sin(th4_arm)*(cos(th3_arm)*sin(th1_arm) - 1.0*cos(th1_arm)*sin(th2_arm)*sin(th3_arm)) - 0.00414*sin(th1_arm) - 0.11945*cos(th1_arm)*cos(th2_arm) + 0.046167919236689778546001150516531*cos(th4_arm)*(cos(th3_arm)*sin(th1_arm) - 1.0*cos(th1_arm)*sin(th2_arm)*sin(th3_arm)) - 0.21144060406275340674442375643594*cos(th1_arm)*cos(th2_arm)*cos(th4_arm) + 0.046167919236689778546001150516531*cos(th1_arm)*cos(th2_arm)*sin(th4_arm);

    //Desired cartesian space position (from mapping above)
    ef_R_home[3*sensor_hand + 0] = 0.00414*cos(th1_arm_des) - 0.11945*cos(th2_arm_des)*sin(th1_arm_des) - 0.046167919236689778546001150516531*cos(th4_arm_des)*(cos(th1_arm_des)*cos(th3_arm_des) + sin(th1_arm_des)*sin(th2_arm_des)*sin(th3_arm_des)) - 0.21144060406275340674442375643594*sin(th4_arm_des)*(cos(th1_arm_des)*cos(th3_arm_des) + sin(th1_arm_des)*sin(th2_arm_des)*sin(th3_arm_des)) + 0.046167919236689778546001150516531*cos(th2_arm_des)*sin(th1_arm_des)*sin(th4_arm_des) - 0.21144060406275340674442375643594*cos(th2_arm_des)*cos(th4_arm_des)*sin(th1_arm_des);
    ef_R_home[3*sensor_hand + 1] = 0.11945*sin(th2_arm_des) + 0.21144060406275340674442375643594*cos(th4_arm_des)*sin(th2_arm_des) - 0.046167919236689778546001150516531*sin(th2_arm_des)*sin(th4_arm_des) - 0.21144060406275340674442375643594*cos(th2_arm_des)*sin(th3_arm_des)*sin(th4_arm_des) - 0.046167919236689778546001150516531*cos(th2_arm_des)*cos(th4_arm_des)*sin(th3_arm_des) - 0.17668;
    ef_R_home[3*sensor_hand + 2] = 0.21144060406275340674442375643594*sin(th4_arm_des)*(cos(th3_arm_des)*sin(th1_arm_des) - 1.0*cos(th1_arm_des)*sin(th2_arm_des)*sin(th3_arm_des)) - 0.00414*sin(th1_arm_des) - 0.11945*cos(th1_arm_des)*cos(th2_arm_des) + 0.046167919236689778546001150516531*cos(th4_arm_des)*(cos(th3_arm_des)*sin(th1_arm_des) - 1.0*cos(th1_arm_des)*sin(th2_arm_des)*sin(th3_arm_des)) - 0.21144060406275340674442375643594*cos(th1_arm_des)*cos(th2_arm_des)*cos(th4_arm_des) + 0.046167919236689778546001150516531*cos(th1_arm_des)*cos(th2_arm_des)*sin(th4_arm_des);

    //End effector  Jacobian WITHOUT  force sensor length
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

    //Cartesian space arm position WITH force sensor length
    // ef_R[0] = 0.00414*cos(th1_arm) - 0.11945*cos(th2_arm)*sin(th1_arm) - 0.044307643397825228266384556263802*cos(th4_arm)*(cos(th1_arm)*cos(th3_arm) + sin(th1_arm)*sin(th2_arm)*sin(th3_arm)) - 0.28991855876216779175358340125968*sin(th4_arm)*(cos(th1_arm)*cos(th3_arm) + sin(th1_arm)*sin(th2_arm)*sin(th3_arm)) + 0.044307643397825228266384556263802*cos(th2_arm)*sin(th1_arm)*sin(th4_arm) - 0.28991855876216779175358340125968*cos(th2_arm)*cos(th4_arm)*sin(th1_arm);
    // ef_R[1] = 0.11945*sin(th2_arm) + 0.28991855876216779175358340125968*cos(th4_arm)*sin(th2_arm) - 0.044307643397825228266384556263802*sin(th2_arm)*sin(th4_arm) - 0.28991855876216779175358340125968*cos(th2_arm)*sin(th3_arm)*sin(th4_arm) - 0.044307643397825228266384556263802*cos(th2_arm)*cos(th4_arm)*sin(th3_arm) - 0.17668;
    // ef_R[2] = 0.28991855876216779175358340125968*sin(th4_arm)*(cos(th3_arm)*sin(th1_arm) - 1.0*cos(th1_arm)*sin(th2_arm)*sin(th3_arm)) - 0.00414*sin(th1_arm) - 0.11945*cos(th1_arm)*cos(th2_arm) + 0.044307643397825228266384556263802*cos(th4_arm)*(cos(th3_arm)*sin(th1_arm) - 1.0*cos(th1_arm)*sin(th2_arm)*sin(th3_arm)) - 0.28991855876216779175358340125968*cos(th1_arm)*cos(th2_arm)*cos(th4_arm) + 0.044307643397825228266384556263802*cos(th1_arm)*cos(th2_arm)*sin(th4_arm);

    //End effector WITH Jacobian 
    // Jc_R[0][0] = 0.28991855876216779175358340125968*sin(th4_arm)*(cos(th3_arm)*sin(th1_arm) - 1.0*cos(th1_arm)*sin(th2_arm)*sin(th3_arm)) - 0.00414*sin(th1_arm) - 0.11945*cos(th1_arm)*cos(th2_arm) + 0.044307643397825228266384556263802*cos(th4_arm)*(cos(th3_arm)*sin(th1_arm) - 1.0*cos(th1_arm)*sin(th2_arm)*sin(th3_arm)) - 0.28991855876216779175358340125968*cos(th1_arm)*cos(th2_arm)*cos(th4_arm) + 0.044307643397825228266384556263802*cos(th1_arm)*cos(th2_arm)*sin(th4_arm);
    // Jc_R[0][1] = 0.11945*sin(th1_arm)*sin(th2_arm) + 0.28991855876216779175358340125968*cos(th4_arm)*sin(th1_arm)*sin(th2_arm) - 0.044307643397825228266384556263802*sin(th1_arm)*sin(th2_arm)*sin(th4_arm) - 0.044307643397825228266384556263802*cos(th2_arm)*cos(th4_arm)*sin(th1_arm)*sin(th3_arm) - 0.28991855876216779175358340125968*cos(th2_arm)*sin(th1_arm)*sin(th3_arm)*sin(th4_arm);
    // Jc_R[0][2] = 0.28991855876216779175358340125968*sin(th4_arm)*(cos(th1_arm)*sin(th3_arm) - 1.0*cos(th3_arm)*sin(th1_arm)*sin(th2_arm)) + 0.044307643397825228266384556263802*cos(th4_arm)*(cos(th1_arm)*sin(th3_arm) - 1.0*cos(th3_arm)*sin(th1_arm)*sin(th2_arm));
    // Jc_R[0][3] = 0.044307643397825228266384556263802*sin(th4_arm)*(cos(th1_arm)*cos(th3_arm) + sin(th1_arm)*sin(th2_arm)*sin(th3_arm)) - 0.28991855876216779175358340125968*cos(th4_arm)*(cos(th1_arm)*cos(th3_arm) + sin(th1_arm)*sin(th2_arm)*sin(th3_arm)) + 0.28991855876216779175358340125968*cos(th2_arm)*sin(th1_arm)*sin(th4_arm) + 0.044307643397825228266384556263802*cos(th2_arm)*cos(th4_arm)*sin(th1_arm);

    // Jc_R[1][0] = 0;
    // Jc_R[1][1] = 0.11945*cos(th2_arm) + 0.28991855876216779175358340125968*cos(th2_arm)*cos(th4_arm) - 0.044307643397825228266384556263802*cos(th2_arm)*sin(th4_arm) + 0.044307643397825228266384556263802*cos(th4_arm)*sin(th2_arm)*sin(th3_arm) + 0.28991855876216779175358340125968*sin(th2_arm)*sin(th3_arm)*sin(th4_arm);
    // Jc_R[1][2] = - 0.044307643397825228266384556263802*cos(th2_arm)*cos(th3_arm)*cos(th4_arm) - 0.28991855876216779175358340125968*cos(th2_arm)*cos(th3_arm)*sin(th4_arm);
    // Jc_R[1][3] = 0.044307643397825228266384556263802*cos(th2_arm)*sin(th3_arm)*sin(th4_arm) - 0.28991855876216779175358340125968*sin(th2_arm)*sin(th4_arm) - 0.044307643397825228266384556263802*cos(th4_arm)*sin(th2_arm) - 0.28991855876216779175358340125968*cos(th2_arm)*cos(th4_arm)*sin(th3_arm);

    // Jc_R[2][0] = 0.11945*cos(th2_arm)*sin(th1_arm) - 0.00414*cos(th1_arm) + 0.044307643397825228266384556263802*cos(th4_arm)*(cos(th1_arm)*cos(th3_arm) + sin(th1_arm)*sin(th2_arm)*sin(th3_arm)) + 0.28991855876216779175358340125968*sin(th4_arm)*(cos(th1_arm)*cos(th3_arm) + sin(th1_arm)*sin(th2_arm)*sin(th3_arm)) - 0.044307643397825228266384556263802*cos(th2_arm)*sin(th1_arm)*sin(th4_arm) + 0.28991855876216779175358340125968*cos(th2_arm)*cos(th4_arm)*sin(th1_arm);
    // Jc_R[2][1] = 0.11945*cos(th1_arm)*sin(th2_arm) - 0.044307643397825228266384556263802*cos(th1_arm)*sin(th2_arm)*sin(th4_arm) + 0.28991855876216779175358340125968*cos(th1_arm)*cos(th4_arm)*sin(th2_arm) - 0.044307643397825228266384556263802*cos(th1_arm)*cos(th2_arm)*cos(th4_arm)*sin(th3_arm) - 0.28991855876216779175358340125968*cos(th1_arm)*cos(th2_arm)*sin(th3_arm)*sin(th4_arm);
    // Jc_R[2][2] = - 0.044307643397825228266384556263802*cos(th4_arm)*(sin(th1_arm)*sin(th3_arm) + cos(th1_arm)*cos(th3_arm)*sin(th2_arm)) - 0.28991855876216779175358340125968*sin(th4_arm)*(sin(th1_arm)*sin(th3_arm) + cos(th1_arm)*cos(th3_arm)*sin(th2_arm));
    // Jc_R[2][3] = 0.28991855876216779175358340125968*cos(th4_arm)*(cos(th3_arm)*sin(th1_arm) - 1.0*cos(th1_arm)*sin(th2_arm)*sin(th3_arm)) - 0.044307643397825228266384556263802*sin(th4_arm)*(cos(th3_arm)*sin(th1_arm) - 1.0*cos(th1_arm)*sin(th2_arm)*sin(th3_arm)) + 0.044307643397825228266384556263802*cos(th1_arm)*cos(th2_arm)*cos(th4_arm) + 0.28991855876216779175358340125968*cos(th1_arm)*cos(th2_arm)*sin(th4_arm);


   // Check determinant of Jacobian to see if we are close to a singularity
    det_Jc[sensor_hand]= Jc_R[0][0] * (Jc_R[1][1] * Jc_R[2][2] - Jc_R[2][1] * Jc_R[1][2]) -
             Jc_R[0][1] * (Jc_R[1][0] * Jc_R[2][2] - Jc_R[1][2] * Jc_R[2][0]) +
             Jc_R[0][2] * (Jc_R[1][0] * Jc_R[2][1] - Jc_R[1][1] * Jc_R[2][0]);

    // Task space velocities matrix-vector multiplication 
    for (i = 0; i < 3; ++i) {
        for (j = 0; j < 4; ++j) {
            def_R[3*sensor_hand + i] += Jc_R[i][j] * dth_all[4*sensor_hand + j];
        }
    }

    // ** Null Space Computation and Code **//
 
// **** J^+ : Jacobian inverse matrix calculation (m < n -> right inverse ) ****//
    // Jc_R_temp = Jc_R * (Jc_R)^T : Transpose Jc_R and multiply by itself
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            Jc_R_temp[i][j] = 0;
            for (k = 0; k < 4; k++) {
                Jc_R_temp[i][j] += Jc_R[i][k] * Jc_R[j][k];
            }
        }
    }
    //Find inverse of inner jacobian product i.e Jc_R_temp
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

    //Final step: Jc_inv = Jc^T * Jc_inner_inv
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            Jc_R_inv[i][j] = 0;
            for (k = 0; k < 4; k++) {
                Jc_R_inv[i][j] += Jc_R[i][k] * Jc_R_inner_inv[j][k];
            }
        }
    }

    //Compute orthonormal basis for the kernel of Jacobian: P = (I - (J^+) * J)
    // P_ortho initialized to be identity 
    for (i = 0; i < 4; i++) {
        tau_joint_des[i] = Kp_arm_des[i]*(th_arm_des_vec[i] - th_arm_vec[i]) + Kd_arm_des[i]*(0 - dth_arm[i]);  
        tau_null_space_task[i] = 0;
        for (j = 0; j < 4; j++) {
            for (k = 0; k < 3; k++) {
                P_ortho[i][j] -= Jc_R_inv[i][k] * Jc_R[k][j];
            }
            tau_null_space_task[i] += P_ortho[i][j] * tau_joint_des[j];
        }
    }

    // ** END Null Space Computation **//

    // Impedance control for right arm: F_arm -> x, y, z
    F_arm[3*sensor_hand + 0] = -Kx_spring*(ef_R[3*sensor_hand + 0] - ef_R_home[3*sensor_hand + 0]) - Kdx_spring*(def_R[3*sensor_hand + 0] - 0);
    F_arm[3*sensor_hand + 1] = -Ky_spring*(ef_R[3*sensor_hand + 1] - ef_R_home[3*sensor_hand + 1]) - Kdy_spring*(def_R[3*sensor_hand + 1] - 0); 
    F_arm[3*sensor_hand + 2] = -Kz_spring*(ef_R[3*sensor_hand + 2] - ef_R_home[3*sensor_hand + 2]) - Kdz_spring*(def_R[3*sensor_hand + 2] - 0);  

    // Perform primary force control matrix-vector multiplication with transpose
    for (i = 0; i < 4; ++i) {
        for (j = 0; j < 3; ++j) {
                tau_arm[4*sensor_hand+ i] += Jc_R[j][i] * F_arm[3*sensor_hand + j];
        }
        // Add subtask onto joint torques
        //tau_arm[4*sensor_hand+ i] += tau_null_space_task[i]
    }

    //Convention change to motor level
    if(sensor_hand == 0){
        tau_arm[0] = -tau_arm[0]; tau_arm[3] = -2.0*tau_arm[3];
    }
    else{
        tau_arm[5] = -tau_arm[5]; tau_arm[6] = -tau_arm[6]; tau_arm[7] = 2.0*tau_arm[7];
    }

    //End of Force Control
    
    //External moment estimation
    int id_0 = 3*sensor_hand + 0;
    int id_1 = 3*sensor_hand + 1;
    int id_2 = 3*sensor_hand + 2;
    r_i[3*sensor_hand + 0] = (CoM[0] - ef_R[3*sensor_hand + 0]); //X
    if(sensor_hand == 0){
        r_i[3*sensor_hand + 1] = (CoM[2] - ef_R[3*sensor_hand + 1]);// The ordering of CoM vec is (x,z,y)
    }
    else{
        r_i[3*sensor_hand + 1] = (CoM[2] - (-ef_R[3*sensor_hand + 1]));// Flip ef_r convention to actual y position
    }
    r_i[3*sensor_hand + 2] = (CoM[1] - ef_R[3*sensor_hand + 2]);
    // M = r x f 
    M_ext[3*sensor_hand + 0] = r_i[id_1]*F_arm[id_2] - r_i[id_2]*F_arm[id_1];
    M_ext[3*sensor_hand + 1] = r_i[id_2]*F_arm[id_0] - r_i[id_0]*F_arm[id_2];
    M_ext[3*sensor_hand + 2] = r_i[id_0]*F_arm[id_1] - r_i[id_1]*F_arm[id_0]; 
}

//IF POSITION CONTROL (i.e NOT TESTING IMP CONTROL)
if(ARM_MODE == POS_CTRL){
    //Enter switching: Force -> Position Control (alpha = 0 -> position mode)
    if((alpha_switch > 0.0) && (t_duration <= t_switch)){
        alpha_switch = 1 - (t_duration/t_switch);
    }
    //Stay Position Control
    else{
        t_start_arm = (t_us / 1000.0);
        alpha_switch = 0.0;
    }
     //Regardless of switch or not keep gain high 
     for(i=0;i<8;i++){Kd_arm[i] = Kd_arm_des[i];} 
}
//ENABLE FORCE CONTROL
else if(ARM_MODE == FORCE_CTRL){
    //Enter Switching: Position -> Force Control (alpha = 1 -> force mode)
    if((alpha_switch < 1.0) && (t_duration <= t_switch)){
        alpha_switch = (t_duration/t_switch);
        for(i=0;i<8;i++){Kd_arm[i] = 1.5;} // While switching keep Kd gain high
    }
    //Stay in force control mode 
    else{
        t_start_arm = (t_us / 1000.0);
        alpha_switch = 1.0;
        if(contactMade[0] < 2.50){
            for(i=0;i<4;i++){Kd_arm[i] = 0.01;} // Once in contact, lower Kd
        }
        else{
        //    for(i=0;i<4;i++){Kd_arm[i] = 0.05;}
            for(i=0;i<4;i++){Kd_arm[i] = dampingArm;}
        }

        if(contactMade[1] < 2.50){
            for(i=0;i<4;i++){Kd_arm[4 + i] = 0.01;} // Once in contact, lower Kd
        }
        else{
        //    for(i=0;i<4;i++){Kd_arm[4 + i] = 0.05;}
            for(i=0;i<4;i++){Kd_arm[4 + i] = dampingArm;}
        }
    }     
}

//Set reference positions for arm 
// motor_ref_ra[0] = th_R_Arm[0];
// motor_ref_ra[1] = th_R_Arm[1];
// motor_ref_ra[2] = th_R_Arm[2];
// motor_ref_ra[3] = 2.0*th_R_Arm[3];

// motor_ref_la[0] =  th_L_Arm[0];
// motor_ref_la[1] =  th_L_Arm[1];
// motor_ref_la[2] =  th_L_Arm[2];
// motor_ref_la[3] =  2.0*th_L_Arm[3];

motor_ref_ra[0] = arm_home_pos[0];
motor_ref_ra[1] = arm_home_pos[1];
motor_ref_ra[2] = arm_home_pos[2];
motor_ref_ra[3] = arm_home_pos[3];

motor_ref_la[0] =  arm_home_pos[4];
motor_ref_la[1] =  arm_home_pos[5];
motor_ref_la[2] =  arm_home_pos[6];
motor_ref_la[3] =  arm_home_pos[7];

//Send torques and desired gains to controller 
for(i = 0; i < 8; i++){
    tau_arm[i] = alpha_switch * tau_arm[i];
}
for(i = 0; i < 8; i++){
    Kp_arm[i] = (1 - alpha_switch) * Kp_arm_des[i];
}

if(contactMade[0] > 2.5){M_ext[0] = 0; M_ext[1] = 0; M_ext[2] = 0;}
if(contactMade[1] > 2.5){M_ext[3] = 0; M_ext[4] = 0; M_ext[5] = 0;}

//IF ARMS NOT STARTED
if(startTrajArm == 0){
    motor_ref_ra[0] = 0;
    motor_ref_ra[1] = 0;
    motor_ref_ra[2] =  0;
    motor_ref_ra[3] = 0;

    motor_ref_la[0] =  0;
    motor_ref_la[1] =  0;
    motor_ref_la[2] =  0;
    motor_ref_la[3] =  0;  

    tau_arm[0] = 0.0; tau_arm[1] = 0.0; tau_arm[2] = 0.0; tau_arm[3] = 0.0;
    tau_arm[4] = 0.0; tau_arm[5] = 0.0; tau_arm[6] = 0.0; tau_arm[7] = 0.0;

    for(i = 0; i < 8; i++){
        Kp_arm[i] = 0;
        Kd_arm[i] = 0;
    }
}
