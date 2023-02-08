#include <math.h>
#include <iostream>


int main(){

    //Joystick joint angles (input)
    float th_j1 = 0.764;
    float th_j2 = -0.570;
    float th_j3 = 0.474;
    float th_j4 = -0.880;

    //Physical lengths
    float L_joy_shoulder_from_body = 0.2105; //middle to shoulder pivot (to the right)
    float L_joy_shoulder_x = 0.1035; //to the right
    float L_joy_shoulder_y = 0.168; //downwards
    float L_joy_arm = 0.313; //upper arm length
    float L_joy_forearm = 0.339; //lower arm length
    float L_joy_hand = 0.078; //width of the hand that goes inwards
    float L_sat_shoulder_from_body = 0.18;
    float L_sat_forearm = 0.2115;
    float L_sat_arm = 0.11945;

    //HTM of end effector in base frame
    float joy_eff[16];
    joy_eff[0] = -sin(th_j1)*sin(th_j2)*sin(th_j3 - th_j4) + cos(th_j1)*cos(th_j3 - th_j4);
    joy_eff[1] = -sin(th_j1)*cos(th_j2);
    joy_eff[2] = sin(th_j1)*sin(th_j2)*cos(th_j3 - th_j4) + sin(th_j3 - th_j4)*cos(th_j1);
    joy_eff[3] = -L_joy_arm*(sin(th_j1)*sin(th_j2)*cos(th_j3) + sin(th_j3)*cos(th_j1)) - L_joy_forearm*(sin(th_j1)*sin(th_j2)*cos(th_j3 - th_j4) + sin(th_j3 - th_j4)*cos(th_j1)) - L_joy_hand*sin(th_j1)*cos(th_j2) + L_joy_shoulder_x*cos(th_j1) + L_joy_shoulder_y*sin(th_j1)*cos(th_j2);
    joy_eff[4] = sin(th_j1)*cos(th_j3 - th_j4) + sin(th_j2)*sin(th_j3 - th_j4)*cos(th_j1);
    joy_eff[5] = cos(th_j1)*cos(th_j2);
    joy_eff[6] = sin(th_j1)*sin(th_j3 - th_j4) - sin(th_j2)*cos(th_j1)*cos(th_j3 - th_j4);
    joy_eff[7] = -L_joy_arm*(sin(th_j1)*sin(th_j3) - sin(th_j2)*cos(th_j1)*cos(th_j3)) - L_joy_forearm*(sin(th_j1)*sin(th_j3 - th_j4) - sin(th_j2)*cos(th_j1)*cos(th_j3 - th_j4)) + L_joy_hand*cos(th_j1)*cos(th_j2) - L_joy_shoulder_from_body + L_joy_shoulder_x*sin(th_j1) - L_joy_shoulder_y*cos(th_j1)*cos(th_j2);
    joy_eff[8] = -sin(th_j3 - th_j4)*cos(th_j2);
    joy_eff[9] = sin(th_j2);
    joy_eff[10] = cos(th_j2)*cos(th_j3 - th_j4);
    joy_eff[11] = -L_joy_arm*cos(th_j2)*cos(th_j3) - L_joy_forearm*cos(th_j2)*cos(th_j3 - th_j4) + L_joy_hand*sin(th_j2) - L_joy_shoulder_y*sin(th_j2);
    joy_eff[12] = 0;
    joy_eff[13] = 0;
    joy_eff[14] = 0;
    joy_eff[15] = 1;

    //Direction of satyrr's elbow
    float sat_elb_dir[3];
    sat_elb_dir[0] = L_joy_forearm*joy_eff[2] - L_joy_shoulder_x + joy_eff[3];
    sat_elb_dir[1] = L_joy_forearm*joy_eff[6] - L_joy_hand + L_joy_shoulder_from_body + L_joy_shoulder_y + joy_eff[7];
    sat_elb_dir[2] = L_joy_forearm*joy_eff[10] + joy_eff[11];

    //Rz is the unit vector from satyrr's elbow to shoulder
    float Rz[3]; 
    Rz[0] = -sat_elb_dir[0];
    Rz[1] = -sat_elb_dir[1];
    Rz[2] = -sat_elb_dir[2];
    float Rz_mag = sqrt(pow(Rz[0], 2) + pow(Rz[1], 2) + pow(Rz[2], 2));
    Rz[0] /= Rz_mag;
    Rz[1] /= Rz_mag;
    Rz[2] /= Rz_mag;

    //Ry is the component of joystick's elbow axis perpendicular to Rz (as a unit vector)
    //Calculated with: norm3d(yaxis_joy_end - Rz*yaxis_joy_end.dot(Rz))
    float Ry[3];
    Ry[0] = -Rz[0]*(Rz[0]*joy_eff[1] + Rz[1]*joy_eff[5] + Rz[2]*joy_eff[9]) + joy_eff[1];
    Ry[1] = -Rz[1]*(Rz[0]*joy_eff[1] + Rz[1]*joy_eff[5] + Rz[2]*joy_eff[9]) + joy_eff[5];
    Ry[2] = -Rz[2]*(Rz[0]*joy_eff[1] + Rz[1]*joy_eff[5] + Rz[2]*joy_eff[9]) + joy_eff[9];
    float Ry_mag = sqrt(pow(Ry[0], 2) + pow(Ry[1], 2) + pow(Ry[2], 2));
    Ry[0] /= Ry_mag;
    Ry[1] /= Ry_mag;
    Ry[2] /= Ry_mag;

    //Rx is the remaining axis to define satyrr's elbow frame, Rx = Ry x Rz
    float Rx[3];
    Rx[0] = Ry[1]*Rz[2] - Ry[2]*Rz[1];
    Rx[1] = -Ry[0]*Rz[2] + Ry[2]*Rz[0];
    Rx[2] = Ry[0]*Rz[1] - Ry[1]*Rz[0];

    //Inverse kinematics of a spherical wrist
    float th_s1 = M_PI - (fmod(atan2(Rz[2], Rz[0]) + M_PI_2, 2*M_PI));
    float th_s2 = M_PI - (fmod(atan2(Rz[1], sqrt(1 - pow(Rz[1], 2))) + M_PI, 2*M_PI));
    float th_s3 = M_PI - (fmod(atan2(-Rx[1], Ry[1]) + M_PI, 2*M_PI));

    //angle between satyrr's upper arm and joystick forearm
    float th_s4 = -acos(Rz[0]*joy_eff[2] + Rz[1]*joy_eff[6] + Rz[2]*joy_eff[10]);
    if(-Ry[0]*(Rz[1]*joy_eff[10] - Rz[2]*joy_eff[6]) + Ry[1]*(Rz[0]*joy_eff[10] - Rz[2]*joy_eff[2]) - Ry[2]*(Rz[0]*joy_eff[6] - Rz[1]*joy_eff[2]) < 0){
        th_s4 *= -1;
    }

    std::cout << th_s1 << std::endl;
    std::cout << th_s2 << std::endl;
    std::cout << th_s3 << std::endl;
    std::cout << th_s4 << std::endl;
}