/*!
 * @file DesiredCommand.h
 * @brief convert keyborad/gamepad command into desired
 * tracjectory for the robot
 */ 

#ifndef DESIREDCOMMAND_H
#define DESIREDCOMMAND_H


#include "cppTypes.h"
#include "body.h"
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include "StateEstimatorContainer.h"

struct rcCommand{

  // buttons
  // int R1;
  // int L1;
  int start;
  bool start_pressed = false;
  // int up;
  // int right;
  // int down;
  // int left;
  // int A;
  // int B;
  // int X;
  // int Y;

  // joystick
  float lx;
  float ly;
  float rx;
  // float ry;
  
};

struct DesiredStateData{

    DesiredStateData() { zero(); }

    // Zero all data
    void zero();
    int mode = 1; // 0 for standby, 1 for standing, 2 for walking
    // Instataneous desired state comman
    Vec12<double> stateDes;
    Vec12<double> pre_stateDes;
  
};

class DesiredStateCommand {
  public:
    // Initialize
    DesiredStateCommand(StateEstimate* _estimate, double _dt){
      stateEstimate = _estimate;
      dt = _dt;
    }
    void getRCcommand();
    void convertToStateCommands();
    void setStateCommands(Vec3<double> v_des, double yaw_rate);
    // void desiredStateTrajectory(int N, Vec10<double> dtVec);
    double deadband(double command, double minVal, double maxVal);
    // These should come from the inferface
    double maxRoll = 0.4;
    double minRoll = -0.4;
    double maxPitch = 0.4;
    double minPitch = -0.4;
    double maxVelX = 2.0;
    double minVelX = -2.0;
    //double maxVelX = 5.0;
    //double minVelX = -5.0;
    double maxVelY = 0.5;
    double minVelY = -0.5;
    double maxTurnRate = 2.0;
    double minTurnRate = -2.0;
    DesiredStateData data;
    rcCommand _rcCommand;

    //~DesiredStateCommand();
  private:
    StateEstimate* stateEstimate;

    Mat12<double> A; // dynamics matrix for discrete time approximation

    double dt; // Control loop time step
};



#endif