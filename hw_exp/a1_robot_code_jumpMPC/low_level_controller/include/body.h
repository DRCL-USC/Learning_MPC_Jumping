/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/
 /*!
  * @file body.h
  * @brief execute motor commands
  */ 
#ifndef _BODY_H_
#define _BODY_H_

#include "../sdk/include/unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include "unitree_joystick.h"
#include <stdio.h>
#include "NatNet/NatNetClientHelper.h"

using namespace std;
using namespace UNITREE_LEGGED_SDK;

class Comm{
  public:
  Comm(): control(LeggedType::A1, LOWLEVEL), 
	  udp()
  {
    control.InitCmdData(cmd);
  }

  void getCmd();
  void UDPRecv();
  void UDPSend();
  Control control;
  UDP udp;
  // UDP udp_rasp; // UDP for multiple PC
  LowCmd cmd;
  LowState state;

  float dt = 0.001;
  //int time = 0;
};

class OptiTrack{
  public:
  OptiTrack();
  ~OptiTrack();

  void Connect(const char* serverIPAddress, const char* localIPAddress);
  void Disconnect();
  void RetrieveOptiTrackData();

private:
  NatNetClient* client;
  std::vector<RigidBodyPose> latestData;
};

namespace a1_robot {

extern LowCmd lowCmd;
extern LowState lowState;
extern xRockerBtnDataStruct _gamepad;

extern Comm comm;
extern OptiTrack optitrack;
extern int motiontime;
extern double dt;
extern float OptiTrack_data[7];

void Send();
void Recv();

void paramInit();
void sendServoCmd();


};




#endif
