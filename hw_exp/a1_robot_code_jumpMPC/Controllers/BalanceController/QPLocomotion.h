#ifndef QP_LOCOMOTION_H
#define QP_LOCOMOTION_H

#include "../include/FootSwingTrajectory.h"
#include "BalanceController.hpp"
#include "../include/cppTypes.h"
#include "../include/ControlFSMData.h"

using Eigen::Array4d;
using Eigen::Array4i;

class QPLocomotion {
  public:
    QPLocomotion();
    //void initialize();

    void run(ControlFSMData& data);
 
    Vec3<double> pBody_des;
    Vec3<double> vBody_des;
    Vec3<double> aBody_des;

    Vec3<double> pBody_RPY_des;
    Vec3<double> vBody_Ori_des;

    Vec3<double> pFoot_des[4];
    Vec3<double> pFoot_act[4];
    Vec3<double> vFoot_des[4];
    Vec3<double> aFoot_des[4];

    Vec3<double> Fr_des[4];

    Vec4<double> contact_state;
     bool climb = false;
  private:
    BalanceController QP;
    void updateQP(ControlFSMData& data);
    double GaitCycleTime;
    //int horizonLength;
    double dt;
    int iterationCounter = 0;
    Vec3<double> f_ff[4];
    Vec4<double> swingTimes;

    FootSwingTrajectory<double> footSwingTrajectories[4];
    bool firstSwing[4];
    //Gait trotting, bounding, walking;
    double swingTimeRemaining[4];
    Mat3<double> Kp, Kd, Kp_stance, Kd_stance;
    
    Vec3<double> world_position_desired;
    Vec3<double> rpy_int;
    Vec3<double> rpy_comp;
    Vec3<double> pFoot[4];

   

    Mat43<double> W; // W = [1, px, py]
    Vec3<double> a; // a = [a_0, a_1, a_2]; z(x,y) = a_0 + a_1x + a_2y
    Vec4<double> pz;
    double ground_pitch;

    bool firstRun = true;
  
};

#endif