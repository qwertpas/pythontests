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


int main(){

    //right arm
    Vector4d thetas_joy(0.826, -0.594, -0.846, -1.358);
    Matrix4d joy_end = joystick_fwdk(thetas_joy);
    Vector4d thetas_sat = satyrr_invk(joy_end);
    std::cout << "joy_end:\n" << joy_end << std::endl;
    std::cout << "thetas_sat:\n" << thetas_sat(0) << std::endl;
}
