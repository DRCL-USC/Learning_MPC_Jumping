#ifndef FSM_State_H
#define FSM_State_H

#include <stdio.h>
#include <fstream>
#include <iostream>
#include "../low_level_controller/include/body.h"
#include "../low_level_controller/include/ControlFSMData.h"
#include "QPLocomotion.h"
#include "ConvexMPCLocomotion.h"
#include "BalanceController.hpp"
#include "../low_level_controller/sdk/include/unitree_legged_sdk/unitree_legged_sdk.h"
//#include "../LeastSquareSolution/LeastSquareSolution.hpp"
#include <vector> // chuong

using namespace UNITREE_LEGGED_SDK;

class FSM_State{
  public:
    FSM_State(ControlFSMData* _controlFSMData);
    
    // void runControl();  // needed when there is other high-level controllers
    void Jump_MPC();// Jumping with MPC
    void QPstand(); // stand with QP controller and switch to mpc locomotion controller
    // void runBalanceController(); // QP controller (depreciated)
    void runQPloco();
    void PDstand(); // stand up with PD controller
    void Landing_test();// test QP for landing
    //double* p_des_in, double* rpy_des_in
          //            double* v_des_in, double* omega_des_in
    //void runLeastSquare(); // least sqaure solution

    // Holds all off the relevant control data
    Control control;
    ControlFSMData* _data;

   // double transitionDuration;  // transition duration time
   // double tStartTransition;    // time transition starts
    //TransitionData transitionData;

    
    // Pre controls safety checks
    bool checkSafeOrientation = false;  // check roll and pitch

    // Post control safety checks
    bool checkPDesFoot = false;          // do not command footsetps too far
    bool checkForceFeedForward = false;  // do not command huge forces
    bool checkLegSingularity = false;    // do not let leg

    // Leg controller command placeholders for the whole robot (3x4 matrices)
   // Mat34<double> jointFeedForwardTorques;  // feed forward joint torques
   // Mat34<double> jointPositions;           // joint angle positions
   // Mat34<double> jointVelocities;          // joint angular velocities
    Mat34<double> footFeedForwardForces;    // feedforward forces at the feet
   // Mat34<double> footPositions;            // cartesian foot positions
   // Mat34<double> footVelocities;           // cartesian foot velocities

    // Footstep locations for next step
    Mat34<double> footstepLocations; 

    // Higher level Robot body controllers
    BalanceController balanceController;
    QPLocomotion QPloco;
    //LeastSquareSolution leastSquareController;
    ConvexMPCLocomotion Cmpc; 
    //ConvexMPCBounding mpcBounding;
    bool initiated = false;
    bool firstrunQP = true; // for QP
    bool runQP = true;
    bool firstwalk = true;

    double init_yaw = 0;
    Vec3<double> init_walk_pos;

    ofstream pos; // p,v,rpy,feedforwardForce
    ofstream data_f; // collect fmpc
    ofstream data_pd; // collect fmpc + force PD
    // ofstream data_fmdc; 
    ofstream data_fcmd;
    ofstream data_ftotal; // collect fmpc + force PD
    ofstream com_act; // actual vcom, pcom, rpy
    ofstream com_des; // p,v,rpy
    ofstream joint_act; // act joint position, velocity, and torque
    ofstream joint_des; // des joint position, velocity, and torque
    ofstream foot_act; // act foot position
    ofstream foot_des; // des foot position
    ofstream torque; // feedforward and cmd torque+ jointPD
    ofstream Voltage; // computed voltage
    ofstream Current; // computed current
    ofstream optitrack;
    ofstream pfeet_optitrack;


    
    // torque des
    vector<vector<double>> tauDes;
    // Position
    vector<vector<double>> qDes;
    // Velocity
    vector<vector<double>> dqDes;
    // Foot position
    vector<vector<double>> pfDes;
    // Foot velocity
    vector<vector<double>> vfDes;

    // Position
    vector<vector<double>> QDes; // for 3D jumping
      // Force
    vector<vector<double>> FDes; 
  

 private:
    // Create the cartesian P gain matrix
    Mat3<double> kpMat;

    // Create the cartesian D gain matrix
    Mat3<double> kdMat;

    double percent;
    double t_convert;

    bool jump_MPC = true;
    bool landing_test=false; // test QP for landing

};


#endif
