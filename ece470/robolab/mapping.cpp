#include <math.h>
#include <Eigen/Dense>
#include <iostream>
using Eigen::Matrix4d;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using namespace std;


float L_joy_shoulder_from_body = 0.2105; //middle to shoulder pivot (to the right)
float L_joy_shoulder_x = 0.1035; //to the right
float L_joy_shoulder_y = 0.168; //downwards
float L_joy_arm = 0.313; //upper arm length
float L_joy_forearm = 0.339; //lower arm length
float L_joy_hand = 0.078; //width of the hand that goes inwards

double L_sat_shoulder_from_body = 0.18;
double L_sat_forearm = 0.2115;
double L_sat_arm = 0.11945;

Matrix4d joystick_fwdk(Vector4d thetas_joy){

    float th1 = thetas_joy[0];
    float th2 = thetas_joy[1];
    float th3 = thetas_joy[2];
    float th4 = thetas_joy[3];

    Matrix4d HTM_BS1; HTM_BS1 <<
        cos(th1), -sin(th1), 0, 0,
        sin(th1), cos(th1), 0, -L_joy_shoulder_from_body,
        0, 0, 1, 0,
        0, 0, 0, 1;

    Matrix4d HTM_S1S2; HTM_S1S2 <<
        1, 0, 0, 0,
        0, cos(th2), -sin(th2), 0,
        0, sin(th2), cos(th2), 0,
        0, 0, 0, 1;

    Matrix4d HTM_S2S3; HTM_S2S3 <<
        cos(th3), 0, sin(th3), L_joy_shoulder_x,
        0, 1, 0, -L_joy_shoulder_y,
        -sin(th3), 0, cos(th3), 0,
        0, 0, 0, 1;

    Matrix4d HTM_S3E; HTM_S3E <<
        cos(-th4), 0, sin(-th4), 0,
        0, 1, 0, 0,
        -sin(-th4), 0, cos(-th4), -L_joy_arm,
        0, 0, 0, 1;

    Matrix4d HTM_EeF; HTM_EeF <<
        1, 0, 0, 0,
        0, 1, 0, L_joy_hand,
        0, 0, 1, -L_joy_forearm,
        0, 0, 0, 1;

    Matrix4d end_effector = HTM_BS1 * HTM_S1S2 * HTM_S2S3 * HTM_S3E * HTM_EeF;

    return end_effector;
}

double ang_wrap(double num){
    return fmod(num + M_PI, 2*M_PI) - M_PI;
}

double ang_betw(Vector3d v1, Vector3d v2, Vector3d axis){
    double ang = acos(v1.dot(v2) / (v1.norm()*v2.norm()));
    if(v1.cross(v2).dot(axis) < 0) ang *= -1;
    return ang;
}

Vector3d spherical_invk(Matrix3d R){
    Matrix3d shift; shift <<
        0, 0, 1,
        1, 0, 0,
        0, 1, 0;
    
    R = shift * R;

    double s5 = sqrt(1 - R(2,2)*R(2,2));

    double theta4 = atan2(R(0,2), R(1,2)) - M_PI_2;
    double theta5 = atan2(R(2,2), s5);
    double theta6 = atan2(-R(2,0), R(2,1));

    Vector3d sol1(-ang_wrap(theta4), -ang_wrap(theta5), -ang_wrap(theta6));
    return sol1;
    //there's another solution but that often has angles near the wraparound
}


Vector4d satyrr_invk(Matrix4d joy_end){

    //extract position and coordinate frame components of joystick end effector
    Vector3d p_joy_end(joy_end(0,3), joy_end(1,3), joy_end(2,3));
    Vector3d yaxis_joy_end(joy_end(0,1), joy_end(1,1), joy_end(2,1));
    Vector3d zaxis_joy_end(joy_end(0,2), joy_end(1,2), joy_end(2,2));

    //find the joystick elbow position by subtracting the forearm
    Vector3d p_joy_elb = p_joy_end - L_joy_forearm * -zaxis_joy_end;

    //find the vector from joystick's shoulder to 
    Vector3d p_shoulder_zero(L_joy_shoulder_x, -L_joy_shoulder_from_body - L_joy_shoulder_y + L_joy_hand, 0);
    Vector3d p_sat_elb = L_sat_arm * (p_joy_elb - p_shoulder_zero).normalized();

    //build the frame at satyrr's elbow so the y axis is the elbow rotation
    Vector3d Rz = -p_sat_elb.normalized();
    Vector3d Ry = (yaxis_joy_end - Rz*yaxis_joy_end.dot(Rz)).normalized();
    Vector3d Rx = Ry.cross(Rz).normalized();
    Matrix3d R_sphere;
    R_sphere << Rx, Ry, Rz;

    //find the values of the 3 joints to achieve the elbow frame
    Vector3d sphere_joints = spherical_invk(R_sphere);

    //find angle between satyrr's upper arm and joystick's forearm
    double theta4 = -ang_betw(zaxis_joy_end, Rz, Ry);

    //combine thetas1-3 and theta4 into 1 vector
    Vector4d thetas_sat;
    thetas_sat << sphere_joints, theta4;

    // std::cout << "p_joy_elb:\n" << p_joy_elb << std::endl;
    // std::cout << "p_sat_elb:\n" << p_sat_elb << std::endl;
    // std::cout << "Rx:\n" << Rx << std::endl;
    // std::cout << "Ry:\n" << Ry << std::endl;
    // std::cout << "Rz:\n" << Rz << std::endl;
    // std::cout << "R_sphere:\n" << R_sphere << std::endl;
    // std::cout << "thetas_sat:\n" << thetas_sat << std::endl;

    return thetas_sat;
}

void map_lw(float th_j1, float th_j2, float th_j3, float th_j4){
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


int main(){

    for(float i1=-0.5; i1<=0.5; i1+=0.2){
        for(float i2=-0.5; i2<=0.5; i2+=0.2){
            for(float i3=-0.5; i3<=0.5; i3+=0.2){
                for(float i4=-0.5; i4<=0.5; i4+=0.2){
                    //right arm
                    // Vector4d thetas_joy(0.826, -0.594, -0.846, -1.358);
                    Vector4d thetas_joy(i1, i2, i3, i4);
                    Matrix4d joy_end = joystick_fwdk(thetas_joy);
                    Vector4d thetas_sat = satyrr_invk(joy_end);
                    // std::cout << "joy_end:\n" << joy_end << std::endl;
                    // std::cout << "thetas_sat:\n" << std::endl;
                    std::cout << thetas_sat(0) << std::endl;
                    std::cout << thetas_sat(1) << std::endl;
                    std::cout << thetas_sat(2) << std::endl;
                    std::cout << thetas_sat(3) << std::endl;

                    std::cout << "-----" << std::endl;

                    map_lw(thetas_joy[0], thetas_joy[1], thetas_joy[2], thetas_joy[3]);

                    std::cout << "=========" << std::endl;

                }
            }
        }
    }
    
}
