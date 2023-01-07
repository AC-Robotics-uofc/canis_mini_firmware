#include "kinematics.h"
#include "robot_constant.h"
#include <math.h>
#include <Arduino.h>

#define RAD_TO_DEG 180.0 / M_PI

void ik() {
    double superior_right_dyz = sqrt(superior_right_y * superior_right_y + superior_right_z * superior_right_z);
    double superior_left_dyz = sqrt(superior_left_y * superior_left_y + superior_left_z * superior_left_z);
    double inferior_right_dyz = sqrt(inferior_right_y * inferior_right_y + inferior_right_z * inferior_right_z);
    double inferior_left_dyz = sqrt(inferior_left_y * inferior_left_y + inferior_left_z * inferior_left_z);

    // ######### Calculation Process Vars ############
    // From this repo: https://github.com/Technician13/QuadrupedInverseKinematic
    // Example Desmos Graph: https://www.desmos.com/calculator/m8do1lkiop
    // Motor positions are stored in extern vars at bottom

    double superior_right_lyz = sqrt(superior_right_dyz * superior_right_dyz - shoulder_length * shoulder_length);
    double superior_left_lyz = sqrt(superior_left_dyz * superior_left_dyz - shoulder_length * shoulder_length);
    double inferior_right_lyz = sqrt(inferior_right_dyz * inferior_right_dyz - shoulder_length * shoulder_length);
    double inferior_left_lyz = sqrt(inferior_left_dyz * inferior_left_dyz - shoulder_length * shoulder_length);

    double superior_right_g1 = atan2(superior_right_y, superior_right_z);
    double superior_left_g1 = atan2(superior_left_y, superior_left_z);
    double inferior_right_g1 = atan2(inferior_right_y, inferior_right_z);
    double inferior_left_g1 = atan2(inferior_left_y, inferior_left_z);

    double superior_right_g2 = atan2(shoulder_length, superior_right_lyz);
    double superior_left_g2 = atan2(shoulder_length, superior_left_lyz);
    double inferior_right_g2 = atan2(shoulder_length, inferior_right_lyz);
    double inferior_left_g2 = atan2(shoulder_length, inferior_left_lyz);

    double superior_right_lxz = sqrt(superior_right_lyz * superior_right_lyz - superior_right_x * superior_right_x);
    double superior_left_lxz = sqrt(superior_left_lyz * superior_left_lyz - superior_left_x * superior_left_x);
    double inferior_right_lxz = sqrt(inferior_right_lyz * inferior_right_lyz - inferior_right_x * inferior_right_x);
    double inferior_left_lxz = sqrt(inferior_left_lyz * inferior_left_lyz - inferior_left_x * inferior_left_x);

    double nmult = 1 / (2 * arm_length);

    double superior_right_n = (superior_right_lxz * superior_right_lxz - forearm_length * forearm_length - arm_length * arm_length) * nmult;
    double superior_left_n = (superior_left_lxz * superior_left_lxz - forearm_length * forearm_length - arm_length * arm_length) * nmult;
    double inferior_right_n = (inferior_right_lxz * inferior_right_lxz - forearm_length * forearm_length - arm_length * arm_length) * nmult;
    double inferior_left_n = (inferior_left_lxz * inferior_left_lxz - forearm_length * forearm_length - arm_length * arm_length) * nmult;

    double superior_right_a1 = atan2(superior_right_x, superior_right_lyz);
    double superior_left_a1 = atan2(superior_left_x, superior_left_lyz);
    double inferior_right_a1 = atan2(inferior_right_x, inferior_right_lyz);
    double inferior_left_a1 = atan2(inferior_left_x, inferior_left_lyz);

    double superior_right_a2 = -acos((arm_length + superior_right_n) / superior_right_lxz);
    double superior_left_a2 = -acos((arm_length + superior_left_n) / superior_left_lxz);
    double inferior_right_a2 = -acos((arm_length + inferior_right_n) / inferior_right_lxz);
    double inferior_left_a2 = -acos((arm_length + inferior_left_n) / inferior_left_lxz);

    superior_right_forearm_extensor_pos = acos(superior_right_n / forearm_length);
    superior_left_forearm_extensor_pos = acos(superior_left_n / forearm_length);
    inferior_right_forearm_extensor_pos = acos(inferior_right_n / forearm_length);
    inferior_left_forearm_extensor_pos = acos(inferior_left_n / forearm_length);

    superior_right_arm_extensor_pos = -(superior_right_a1 + superior_right_a2);
    superior_left_arm_extensor_pos = -(superior_left_a1 + superior_left_a2);
    inferior_right_arm_extensor_pos = -(inferior_right_a1 + inferior_right_a2);
    inferior_left_arm_extensor_pos = -(inferior_left_a1 + inferior_left_a2);

    superior_right_shoulder_abductor_pos = (-(superior_right_g1 + superior_right_g2)) + M_PI;
    superior_left_shoulder_abductor_pos = (-(superior_left_g1 + superior_left_g2)) + M_PI;
    inferior_right_shoulder_abductor_pos = (-(inferior_right_g1 + inferior_right_g2)) + M_PI;
    inferior_left_shoulder_abductor_pos = (-(inferior_left_g1 + inferior_left_g2)) + M_PI;
}

void fk() {

}