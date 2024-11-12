#ifndef CONVEXMPCLOCOMOTION_H
#define CONVEXMPCLOCOMOTION_H

#include "../include/FootSwingTrajectory.h"
#include "../include/ControlFSMData.h"
#include "../include/cppTypes.h"
#include <fstream>
#include <vector> // Chuong add
#include "common_types.h"
#include <qpOASES.hpp>
#include <torch/script.h>
#include <torch/torch.h>

using Eigen::Array4f;
using Eigen::Array4i;
using Eigen::Array4d;


struct CMPC_Result {
  LegControllerCommand commands[4];
  Vec4<float> contactPhase;
};


class Gait
{
public:
  Gait(int nMPC_segments, Vec4<int> offsets, Vec4<int>  durations, const std::string& name="");
  ~Gait();
  Vec4<double> getContactSubPhase();
  Vec4<double> getSwingSubPhase();
  int* mpc_gait();
  void setIterations(int iterationsPerMPC, int currentIteration);
  int _stance;
  int _swing;


private:
  int _nMPC_segments;
  int* _mpc_table;
  Array4i _offsets; // offset in mpc segments
  Array4i _durations; // duration of step in mpc segments
  Array4d _offsetsPhase; // offsets in phase (0 to 1)
  Array4d _durationsPhase; // durations in phase (0 to 1)
  int _iteration;
  int _nIterations;
  double _phase;

};


class ConvexMPCLocomotion {
public:
  ConvexMPCLocomotion(double _dt, int _iterations_between_mpc, int _iterations_between_mpc_flight);
  void initialize();

  void run(ControlFSMData& data);
  void run_Jump(ControlFSMData& data, vector<vector<double>> QDes, vector<vector<double>> FDes, vector<vector<double>> pfDes);
  // void run_Jump(ControlFSMData& data);
  void setGaitNum(int gaitNum){gaitNumber = gaitNum % 6; if(gaitNum%6 ==0) gaitNumber = 6; return;}

  Vec3<double> pBody_des;
  Vec3<double> vBody_des;
  Vec3<double> aBody_des;

  Vec3<double> pBody_RPY_des;
  Vec3<double> vBody_Ori_des;

  Vec3<double> pFoot_des[4];
  Vec3<double> vFoot_des[4];
  Vec3<double> aFoot_des[4];

  Vec3<double> Fr_des[4];

  Vec4<double> contact_state;
  ofstream time_record, foot_position;

  bool climb = 0;

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
  vector<vector<double>> QDes; 
  // Force
  vector<vector<double>> FDes; 

  int N_TrajRef=1300;
  int dc=500;
  int rc=300;
  int fl=500;

  torch::jit::script::Module module_gnet, module_f1net, module_f2net;
  at::Tensor atOutputg, atOutputf1, atOutputf2;


  double t_mpc;
  double t_nn; // reading time of feedforward Neural Network


  // int N_TrajRef=500;
  // int dc=500;



private:
  void updateMPCIfNeeded(int* mpcTable, ControlFSMData& data, bool omniMode);
  // void updateMPCIfNeeded_Jump(int* mpcTable, ControlFSMData& data, bool omniMode);  
  void updateMPCIfNeeded_Jump(int* mpcTable, ControlFSMData& data, bool omniMode, vector<vector<double>> QDes, vector<vector<double>> FDes, vector<vector<double>> pfDes); 
  int iterationsBetweenMPC, iterationsBetweenMPC_flight;
  int horizonLength;
  int updateMPCfreq;
  double dt;
  double dtMPC, dtMPC_fl;
  int iterationCounter = 0;
  Vec3<double> f_ff[4];
  Vec3<double> f_MPC_b[4]; // force in body frame
  Vec3<double> f_MPC[4];
  Vec3<double> Delta_f[4];
  Vec4<double> swingTimes;
  FootSwingTrajectory<double> footSwingTrajectories[4];
  Gait trotting, bounding, pacing, walking, galloping, pronking;
  Mat3<double> Kp, Kd, Kp_stance, Kd_stance;
  bool firstRun = true;
  bool firstSwing[4];
  double swingTimeRemaining[4];
  double stand_traj[6];
  int current_gait;
  int gaitNumber;
  int current_MPC;

  Vec3<double> world_position_desired;
  Vec3<double> rpy_int;
  Vec3<double> rpy_comp;
  Vec3<double> pFoot[4];
  CMPC_Result result;
  double trajAll[12*36];
  double trajAll_Jump[12*20];
  double g_NN[12*10]; // output of Neural network g has size of (3*4)
  double f1_NN[3*10]; // output of Neural network f1 has size of (3)
  double f2_NN[3*10]; // output of Neural network g has size of (3)
  double pf2comw_ref[4*20];// for 20 horizon
  double FrefAll_Jump[12*20];
  double gait_jumpTable[4*10]; // define contact state in 10 horizon
    double Fjump_des[12];// reference Force from optimization[]
  
  
  Mat43<double> W; // W = [1, px, py]
  Vec3<double> a; // a = [a_0, a_1, a_2]; z(x,y) = a_0 + a_1x + a_2y
  Vec4<double> pz;
  double ground_pitch;
};


#endif //CONVEXMPCLOCOMOTION_H
