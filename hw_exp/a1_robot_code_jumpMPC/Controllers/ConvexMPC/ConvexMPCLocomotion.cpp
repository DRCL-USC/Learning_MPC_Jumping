#include <iostream>
#include "../include/Utilities/Timer.h"
#include "../include/Math/orientation_tools.h"
#include "ConvexMPCLocomotion.h"
#include "convexMPC_interface.h"
#include "../include/body.h"
#include "../include/LegController.h"
#include <eigen3/Eigen/Dense>

using namespace ori;
using namespace UNITREE_LEGGED_SDK;

/* ========================= GAIT ========================= */
Gait::Gait(int nMPC_segments, Vec4<int> offsets, Vec4<int> durations, const std::string& name) :
  _offsets(offsets.array()),
  _durations(durations.array()),
  _nIterations(nMPC_segments)
{
    _mpc_table = new int[nMPC_segments * 4];

    _offsetsPhase = offsets.cast<double>() / (double) nMPC_segments;
    _durationsPhase = durations.cast<double>() / (double) nMPC_segments;

    _stance = durations[0];
    _swing = nMPC_segments - durations[0];
}

Gait::~Gait()
{
    delete[] _mpc_table;
}

Vec4<double> Gait::getContactSubPhase()
{
  Array4d progress = _phase - _offsetsPhase;

  for(int i = 0; i < 4; i++)
  {
    if(progress[i] < 0) progress[i] += 1.;
    if(progress[i] > _durationsPhase[i])
    {
      progress[i] = 0.;
    }
    else
    {
      progress[i] = progress[i] / _durationsPhase[i];
    }
    
  }

  return progress.matrix();
}

Vec4<double> Gait::getSwingSubPhase()
{
  Array4d swing_offset = _offsetsPhase + _durationsPhase;
  for(int i = 0; i < 4; i++)
    if(swing_offset[i] > 1) swing_offset[i] -= 1.;
  Array4d swing_duration = 1. - _durationsPhase;

  Array4d progress = _phase - swing_offset;

  for(int i = 0; i < 4; i++)
  {
    if(progress[i] < 0) progress[i] += 1.;
    if(progress[i] > swing_duration[i])
    {
      progress[i] = 0.;
    }
    else
    {
      progress[i] = progress[i] / swing_duration[i];
    }
  }

  return progress.matrix();
}

int* Gait::mpc_gait()
{
  for(int i = 0; i < _nIterations; i++)
  {
    int iter = (i + _iteration + 1) % _nIterations;
    Array4i progress = iter - _offsets;
    for(int j = 0; j < 4; j++)
    {
      if(progress[j] < 0) progress[j] += _nIterations;
      if(progress[j] < _durations[j])
        _mpc_table[i*4 + j] = 1;
      else
        _mpc_table[i*4 + j] = 0;
    }
  }
  //std::cout << "horizon: " << _iteration << std::endl;
  return _mpc_table;
}

void Gait::setIterations(int iterationsPerMPC, int currentIteration)
{
  _iteration = (currentIteration / iterationsPerMPC) % _nIterations;
  _phase = (double)(currentIteration % (iterationsPerMPC * _nIterations)) / (double) (iterationsPerMPC * _nIterations);
}

/* =========================== Controller ============================= */
ConvexMPCLocomotion::ConvexMPCLocomotion(double _dt, int _iterations_between_mpc, int _iterations_between_mpc_flight) :
  iterationsBetweenMPC(_iterations_between_mpc),
  iterationsBetweenMPC_flight(_iterations_between_mpc_flight),
  horizonLength(10),
  updateMPCfreq(25),
  dt(_dt),
  galloping(horizonLength, Vec4<int>(0,2,7,9),Vec4<int>(5,5,5,5),"Galloping"),
  pronking(horizonLength, Vec4<int>(0,0,0,0),Vec4<int>(4,4,4,4),"Pronking"),
  trotting(horizonLength, Vec4<int>(0,5,5,0), Vec4<int>(5,5,5,5),"Trotting"),
  bounding(horizonLength, Vec4<int>(5,5,0,0),Vec4<int>(5,5,5,5),"Bounding"),
  walking(horizonLength, Vec4<int>(0,3,5,8), Vec4<int>(5,5,5,5), "Walking"),
  pacing(horizonLength, Vec4<int>(5,0,5,0),Vec4<int>(5,5,5,5),"Pacing")
{

      try {

        Timer tff;
        tff.start();
        // // torch::jit::script::Module module;
        module_gnet = torch::jit::load("/home/chuongusc/Documents/GitHub/Learning_MPC_Jumping/trained_model/model_g1.pt");
        module_f1net = torch::jit::load("/home/chuongusc/Documents/GitHub/Learning_MPC_Jumping/trained_model/model_h1.pt");
        module_f2net = torch::jit::load("/home/chuongusc/Documents/GitHub/Learning_MPC_Jumping/trained_model/model_h2.pt");
    //          torch::Device device(torch::kCPU);
    //          module_Chuongnet.to(device);
        
        
        std::cout << "Loading is done........" << std::endl;
        torch::Tensor test_input = torch::tensor({0.0, 0.15, 0.0, 0.2,  -0.15,  -0.2, -0.15}, {torch::kFloat32});//.requires_grad_(true);
        std::cout << "test_input:" << std::endl << test_input << "\n";
        std::vector<torch::jit::IValue> test_inputs;
        test_inputs.push_back(test_input);
        
        for (int i=0; i< 200; i++){

          try {
                // std::cout << Chuong_input.device() << std::endl;
                auto atoutput_g = module_gnet.forward(test_inputs).toTensor();
                std::cout << "OUTPUT g" << std::endl << atoutput_g << "\n";
                auto atoutput_f1 = module_f1net.forward(test_inputs).toTensor();
                std::cout << "OUTPUT f1" << std::endl << atoutput_f1 << "\n";
                auto atoutput_f2 = module_f2net.forward(test_inputs).toTensor();
                std::cout << "OUTPUT atoutputChuong" << std::endl << atoutput_f2 << "\n";
          }

          catch (std::exception &e)
            {
                std::cout << e.what() << std::endl;
            }

        }

        printf("Warming up time %f ms\n", tff.getMs());


    }
    catch (std::exception &e)
    {
        std::cout << e.what() << std::endl;
    }

  gaitNumber = 4;
  dtMPC = dt * iterationsBetweenMPC;
  //std::cout << "dtMPC: " << dtMPC << std::endl;
  rpy_int[2] = 0;
  for(int i = 0; i < 4; i++)
    firstSwing[i] = true;

  foot_position.open("foot_pos.txt");
  time_record.open("time.txt");
}

void ConvexMPCLocomotion::run_Jump(ControlFSMData& data, vector<vector<double>> QDes, vector<vector<double>> FDes, vector<vector<double>> pfDes)
{
  bool omniMode = false;


//  auto* debugSphere = data.visualizationData->addSphere();
//  debugSphere->color = {1,1,1,0.5};
//  debugSphere->radius = 1;
 // data._legController->updateData();
 // data._stateEstimator->run();
  auto& seResult = data._stateEstimator->getResult();
  auto& stateCommand = data._desiredStateCommand;
  //std::cout << "in side mpc" << seResult.rBody << std::endl;;

  // pick gait
  Gait* gait = &trotting;
  if(gaitNumber == 1)
    gait = &bounding;
  else if(gaitNumber == 2)
    gait = &trotting;
  else if(gaitNumber == 3)
    gait = &walking;
  else if(gaitNumber == 4)
    gait = &pacing;
  else if(gaitNumber == 5)
    gait = &galloping;
  else if(gaitNumber == 6)
    gait = &pronking;
  current_gait = gaitNumber;
  // integrate position setpoint
  Vec3<double> v_des_robot(stateCommand->data.stateDes(6), stateCommand->data.stateDes(7),0);
  Vec3<double> v_des_world;
  //v_des_world = coordinateRotation(CoordinateAxis::Z, seResult.rpy[2]).transpose() * v_des_robot;
  //Vec3<double> v_robot = coordinateRotation(CoordinateAxis::Z, seResult.rpy[2])*seResult.vWorld;

  v_des_world = seResult.rBody.transpose() * v_des_robot;
  Vec3<double> v_robot = seResult.vWorld;

  world_position_desired[0] += dt * v_des_world[0];
  world_position_desired[1] += dt * v_des_world[1];
  world_position_desired[2] = 0.3;
  
 // printf("p_des \t%.6f\n", dt * v_des_world[0]);
 //Integral-esque pitch and roll compensation
  if(fabs(v_robot[0]) > .2)   //avoid dividing by zero
  {
    rpy_int[1] += dt*(stateCommand->data.stateDes[4] /*-hw_i->state_estimator->se_ground_pitch*/ - seResult.rpy[1])/v_robot[0];
  }
  if(fabs(v_robot[1]) > 0.1)
  {
    rpy_int[0] += dt*(stateCommand->data.stateDes[3] /*-hw_i->state_estimator->se_ground_pitch*/
        - seResult.rpy[0])/v_robot[1];
  }

  rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);
  rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
  rpy_comp[1] = v_robot[0] * rpy_int[1];
  rpy_comp[0] = v_robot[1] * rpy_int[0] * (gaitNumber!=6);  //turn off for pronking

  


  for(int i = 0; i < 4; i++) {
   pFoot[i] = seResult.position + seResult.rBody.transpose() 
            * (data._quadruped->getHipLocation(i) + data._legController->data[i].p);
   // pFoot[i] = data._legController->data[i].p;
    //std::cout << "pFoot" << i << "\n" << pFoot[i] << std::endl;
  }
  

  // some first time initialization
  if(firstRun)
  {
    std::cout << "Run MPC" << std::endl;
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.position[2];

    Vec3<double> v_des_robot(0,0,0);  // connect to desired state command later
    Vec3<double> v_des_world(0,0,0);  // connect to desired state command later

    Vec3<double> v_robot = seResult.vWorld;
    pBody_des[0] = world_position_desired[0];
    pBody_des[1] = world_position_desired[1];
    pBody_des[2] = world_position_desired[2];
    vBody_des[0] = v_des_world[0];
    vBody_des[1] = v_des_world[1];
    vBody_des[2] = 0;

    pBody_RPY_des[0] = 0;
    pBody_RPY_des[1] = 0;
    pBody_RPY_des[2] = 0; // seResult.rpy[2];

    vBody_Ori_des[0] = 0;
    vBody_Ori_des[1] = 0;
    vBody_Ori_des[2] = 0; // set this for now

    for(int i = 0; i < 4; i++){
      footSwingTrajectories[i].setHeight(0.08);
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);
      //std::cout << "orig foot pos " << i << pFoot[i] << std::endl;
    }
    firstRun = false;
  }

  // contact_state = gait->getContactSubPhase();

 
   // std::cout << "foot force 0 :" << lowState.footForce[0] << std::endl;
   // std::cout << "contact 0 : " << contact_state[0] << std::endl;

  //std::cout << "Swing Time" << swingTimes << std::endl;
  double side_sign[4] = {-1, 1, -1, 1};
  double interleave_y[4] = {-0.08, 0.08, 0.01, -0.01};
  double interleave_gain = -0.2;
  double v_abs = std::fabs(seResult.vBody[0]);

  // calc gait
  // gait->setIterations(iterationsBetweenMPC, iterationCounter);
  
  // load LCM leg swing gains
  // Kp << 700, 0, 0,
  //     0, 700, 0,
  //     0, 0, 350;
  // Kp_stance = 0*Kp;

  // Kd << 15, 0, 0,
  //     0, 15, 0,
  //     0, 0, 15;
  // Kd_stance << 10, 0, 0,
  //              0, 10, 0,
  //              0, 0, 10;
  // gait
  // Vec4<double> contactStates = gait->getContactSubPhase();
  // Vec4<double> swingStates = gait->getSwingSubPhase();
  int* mpcTable = gait->mpc_gait();

  t_nn = 0; t_mpc = 0;
  updateMPCIfNeeded_Jump(mpcTable, data, omniMode, QDes, FDes, pfDes);
  time_record << t_nn << " " << t_mpc << std::endl;
  
  iterationCounter++;

  for(int foot = 0; foot < 4; foot++)
  {
     data._legController->commands[foot].feedforwardForce = f_MPC_b[foot]; // in leg coordinates
     data._legController->commands[foot].force_MPC = f_MPC[foot]; // in world coordinate
     data._legController->commands[foot].deltaForce = Delta_f[foot];

  }
  
  // se->set_contact_state(se_contactState); todo removed
  
  //data._legController->updateCommand();
}

void ConvexMPCLocomotion::updateMPCIfNeeded_Jump(int* mpcTable, ControlFSMData& data, bool omniMode, vector<vector<double>> QDes, vector<vector<double>> FDes, vector<vector<double>> pfDes) 
{
  //iterationsBetweenMPC = 30;
  std::cout << "iterationCounter: "<< iterationCounter << std::endl;
  std::cout << "iterationsBetweenMPC: "<< iterationsBetweenMPC << std::endl;
  std::cout << "iterationsBetweenMPC_flight: "<< iterationsBetweenMPC_flight << std::endl;
  // std::cout << "MPC iteration: "<< iterationCounter % iterationsBetweenMPC << std::endl;
  // std::cout << "horizon length:" << horizonLength << std::endl;
  // std::cout << "updateMPCfreq:" << updateMPCfreq << std::endl;
  // std::cout << "N_TrajRef: "<< N_TrajRef << std::endl;

  // if((iterationCounter % iterationsBetweenMPC) == 0 && iterationCounter<=N_TrajRef){
  //   //std::cout << "size qDes is: " << QDes[0].size() << std::endl;
  //   int idx = iterationCounter;
  //   std::cout << "qDes is: " << QDes[3][idx] << std::endl;  
  //   std::cout << "FDes is: " << FDes[3][idx] << std::endl;  

  // }

  if((iterationCounter % updateMPCfreq) == 0)
  {
    std::cout << "-----------------------" << std::endl;

    dtMPC = dt * iterationsBetweenMPC;

    int N1 = (800 - iterationCounter) / (1000 * dtMPC);
    cout << "N1:"<< N1 << endl;
    int N2 = horizonLength - N1;

    int idx_pred = 0;

    // int current_MPC=iterationCounter/25; // 25 is set at the beginning of FSM.cpp
    // std::cout << "current_MPC: "<< current_MPC << std::endl;

   // std::cout << "runMPC" << std::endl;
    auto seResult = data._stateEstimator->getResult();
    auto& stateCommand = data._desiredStateCommand;

    std::cout << stateCommand->data.stateDes(6) << std::endl;
    std::cout << stateCommand->data.stateDes(7) << std::endl;
    double* p = seResult.position.data();
    double* v = seResult.vWorld.data();
    double* w = seResult.omegaWorld.data();
    double* q = seResult.orientation.data();

    // float* pf = (float*) seResult.position.data();
    // float* vf = (float*) seResult.vWorld.data();
    // float* wf = (float*) seResult.omegaWorld.data();
    // float* qf = (float*) seResult.orientation.data();

    double r[12];
    for(int i = 0; i < 12; i++)
      r[i] = pFoot[i%4][i/4] - seResult.position[i/4];

    // double Q[12] = {0.25, 50, 0.25, 2.5, 2.5, 30, 0, 0.1, 0, 0.2, 0.2, 0.2}; // try 1 // rpy, p, w, v 
    // double Q[12] = {0.25, 50, 0.25, 10, 0.25, 10, 0, 0.1, 0, 10, 0.2, 0.2}; // try 2: 
    // double Q[12] = {0.25, 50, 0.25, 20, 0.25, 20, 0, 10, 0, 20, 0.2, 20}; // try 3: not good as try 2, robot jump shorter


    // double Q[12] = {0.25, 50, 0.25, 2.5, 2.5, 2.5, 0, 0.1, 0, 50, 0.2, 0.2}; // try 4 // rpy, p, w, v 
    // double Q[12] = {0.25, 50, 0.25, 50, 2.5, 2.5, 0, 0.1, 0, 50, 0.2, 0.2}; // try 5 // rpy, p, w, v 
    // double Q[12] = {0.25, 50, 0.25, 50, 2.5, 2.5, 0, 10, 0, 10, 0.2, 0.2}; // try 6 // rpy, p, w, v 

    double Q[12] = {0.25, 50, 0.25, 50, 2.5, 2.5, 0, 10, 0, 10, 0.2, 0.2}; // try 6 // rpy, p, w, v 

    // double Q[12] = {0.25, 30, 0.25, 20, 2.5, 2.5, 0, 5, 0, 5, 0.2, 0.2}; // try 7 // rpy, p, w, v // not better


    double pitch = seResult.rpy[1];

    // double Q[12] = {0.25, 0.5, 20, 1.0, 1.0, 15, 0, 0, 0.3, 0.2, 0.2, 0.2}; // original
    // double yaw = seResult.rpy[2];
    double* weights = Q;
    double alpha = 1e-6; // make setting eventually

    //printf("current posistion: %3.f %.3f %.3f\n", p[0], p[1], p[2]);

    if(alpha > 1e-4)
    {

      std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
      alpha = 1e-5;
    }
    Vec3<double> v_des_robot(stateCommand->data.stateDes[6], stateCommand->data.stateDes[7],0);
    //Vec3<double> v_des_world = coordinateRotation(CoordinateAxis::Z, seResult.rpy[2]).transpose() * v_des_robot;

    Vec3<double> v_des_world = seResult.rBody.transpose() * v_des_robot;
    //float trajInitial[12] = {0,0,0, 0,0,.25, 0,0,0,0,0,0};

    
      const double max_pos_error = .1;
      double xStart = world_position_desired[0];
      double yStart = world_position_desired[1];
      //std::cout << "orig " << xStart << "  " << yStart << std::endl;
      //printf("orig \t%.6f\t%.6f\n", xStart, yStart);
      //printf("ref: \t%.6f\t%.6f\n", p[0], p[1]);

      if(xStart - p[0] > max_pos_error) xStart = p[0] + max_pos_error;
      if(p[0] - xStart > max_pos_error) xStart = p[0] - max_pos_error;

      if(yStart - p[1] > max_pos_error) yStart = p[1] + max_pos_error;
      if(p[1] - yStart > max_pos_error) yStart = p[1] - max_pos_error;

      world_position_desired[0] = xStart;
      world_position_desired[1] = yStart;

      //printf("xys: \t%.3f\t%3.f\n", xStart, yStart);
      //printf("perr \t%.3f\t%.3f\n", p[0] - world_position_desired[0], p[1] - world_position_desired[1]);
      
      double trajInitial[12] = {rpy_comp[0], //+ stateCommand->data.stateDes[3],  // 0
                                rpy_comp[1], //+ stateCommand->data.stateDes[4],    // 1
                                stateCommand->data.stateDes[5],    // 2
                                xStart,                                   // 3
                                yStart,                                   // 4
                                0.3 ,   // 5
                                0,                                        // 6
                                0,                                        // 7
                                stateCommand->data.stateDes[11],  // 8
                                v_des_world[0],                           // 9
                                v_des_world[1],                           // 10
                                0};                                       // 11

    for (int i=0; i<12*20; i++){
        trajAll_Jump[i] = 0;
        FrefAll_Jump[i] = 0;
    }
    // current coordinates
    double trajInitial_Jump[12] =  {seResult.rpy[0],                              // 0 roll
                                      seResult.rpy[1],                              // 1 pitch
                                      seResult.rpy[2],                              // 2 yaw
                                      p[0],                                   // 3  px
                                      p[1],                                   // 4  py
                                      p[2] ,   // 5 pz
                                      w[0],                                        // 6 wx
                                      w[1],                                        // 7 wy
                                      w[2],  // 8 wz
                                      v[0],                           // 9 vx
                                      v[1],                           // 10 vy
                                      v[2]};     
    
    for(int j = 0; j < 12; j++)
          trajAll_Jump[j] = trajInitial_Jump[j]; // The first column of trajAll_Jump is always trajInitial_Jump

    if (iterationCounter<=dc){
          for (int j=0; j<4; j++)
            gait_jumpTable[j]=1;
    }
    else if(iterationCounter>dc && iterationCounter<= dc+rc){
          gait_jumpTable[0]=0; gait_jumpTable[1]=0;
          gait_jumpTable[2]=1; gait_jumpTable[3]=1;
    }
    else{
          for (int j=0; j<4; j++)
            gait_jumpTable[j]=0;
    }

  // Vec12<double> Fjump_des;
    double Fjump_Initial[12]={0,0,0,0,0,0,0,0,0,0,0,0};


    if (iterationCounter <=N_TrajRef){
      Fjump_Initial[0]=FDes[0][iterationCounter]/2;
      Fjump_Initial[2]=FDes[1][iterationCounter]/2;
      Fjump_Initial[3]=FDes[0][iterationCounter]/2;
      Fjump_Initial[5]=FDes[1][iterationCounter]/2;
      Fjump_Initial[6]=FDes[2][iterationCounter]/2;
      Fjump_Initial[8]=FDes[3][iterationCounter]/2;
      Fjump_Initial[9]=FDes[2][iterationCounter]/2;
      Fjump_Initial[11]=FDes[3][iterationCounter]/2;
    } 
    else{
      for (int k=0; k<12;k++){
        Fjump_Initial[k]=0;
      }
    }

    for(int j = 0; j < 12; j++)
        FrefAll_Jump[j] = Fjump_Initial[j]; // The first column of trajAll_Jump is always trajInitial_Jump
     
    if (N1 < horizonLength && N1 >0){ // in hybrid prediction mode:
          for (int i = 1; i < horizonLength; i++){
              if (i<= N1){
                idx_pred = iterationCounter+i*iterationsBetweenMPC; // contact prediction
              }
              if (i > N1){
                idx_pred = iterationCounter+N1*iterationsBetweenMPC+(i-N1)*iterationsBetweenMPC_flight; // flight prediction
                if (idx_pred >= N_TrajRef)
                  idx_pred = N_TrajRef;
              }
              cout << "i = " << i << "," << "idx_pred:" << idx_pred << endl;
              // get trajectory reference
              trajAll_Jump[12*i+1]=QDes[2][idx_pred]; // pitch
              trajAll_Jump[12*i+3]=QDes[0][idx_pred]; // px
              trajAll_Jump[12*i+5]=QDes[1][idx_pred] + 0.15; // pz, 0.15 is initial height in Matlab
              trajAll_Jump[12*i+7]=QDes[9][idx_pred]; // wy
              trajAll_Jump[12*i+9]=QDes[7][idx_pred]; // vx
              trajAll_Jump[12*i+11]=QDes[8][idx_pred]; // vz

              // // get force reference
              FrefAll_Jump[12*i+0]=FDes[0][idx_pred]/2; // FRx
              FrefAll_Jump[12*i+2]=FDes[1][idx_pred]/2; // FRz
              FrefAll_Jump[12*i+3]=FDes[0][idx_pred]/2; // FLx
              FrefAll_Jump[12*i+5]=FDes[1][idx_pred]/2; // FLz
              FrefAll_Jump[12*i+6]=FDes[2][idx_pred]/2; // RRx
              FrefAll_Jump[12*i+8]=FDes[3][idx_pred]/2; // RRz
              FrefAll_Jump[12*i+9]=FDes[2][idx_pred]/2; // RLx
              FrefAll_Jump[12*i+11]=FDes[3][idx_pred]/2; // RLz

              if (idx_pred > dc+rc){
                  for (int j=0; j<12; j++){
                       FrefAll_Jump[12*i+j]=0; 
                  }
              }
              // get contact schedule reference
              if (idx_pred<=dc){
                  for (int j=0; j<4; j++){
                      gait_jumpTable[4*i+j]=1;
                  }
              }
              else if(idx_pred>dc && idx_pred<= dc+rc){
                  gait_jumpTable[4*i+0]=0; 
                  gait_jumpTable[4*i+1]=0;
                  gait_jumpTable[4*i+2]=1; 
                  gait_jumpTable[4*i+3]=1;
              }
              else{
                  for (int j=0; j<4; j++){
                      gait_jumpTable[4*i+j]=0;
                  }

              }

          }
    }
    else{ // totally in either flight or contact phase
            for(int i = 1; i < horizonLength; i++)
            {
              if (N1>0){
                idx_pred = iterationCounter+i*iterationsBetweenMPC;
              }
              else{
                idx_pred = iterationCounter+i*iterationsBetweenMPC_flight;
              }
              // idx_pred = iterationCounter+i*iterationsBetweenMPC;
              if (idx_pred>=N_TrajRef){
                idx_pred = N_TrajRef;
              }
              cout << "i = " << i << "," << "idx_pred:" << idx_pred << endl;

              // get trajectory reference
              trajAll_Jump[12*i+1]=QDes[2][idx_pred]; // pitch
              trajAll_Jump[12*i+3]=QDes[0][idx_pred]; // px
              trajAll_Jump[12*i+5]=QDes[1][idx_pred]+0.15; // pz, 0.15 is initial height in Matlab
              trajAll_Jump[12*i+7]=QDes[9][idx_pred]; // wy
              trajAll_Jump[12*i+9]=QDes[7][idx_pred]; // vx
              trajAll_Jump[12*i+11]=QDes[8][idx_pred]; // vz
            
              // get force reference
              FrefAll_Jump[12*i+0]=FDes[0][idx_pred]/2; // FRx
              FrefAll_Jump[12*i+2]=FDes[1][idx_pred]/2; // FRz
              FrefAll_Jump[12*i+3]=FDes[0][idx_pred]/2; // FLx
              FrefAll_Jump[12*i+5]=FDes[1][idx_pred]/2; // FLz
              FrefAll_Jump[12*i+6]=FDes[2][idx_pred]/2; // RRx
              FrefAll_Jump[12*i+8]=FDes[3][idx_pred]/2; // RRz
              FrefAll_Jump[12*i+9]=FDes[2][idx_pred]/2; // RLx
              FrefAll_Jump[12*i+11]=FDes[3][idx_pred]/2; // RLz

              // get contact schedule reference
              if (idx_pred<=dc){
                  for (int j=0; j<4; j++){
                      gait_jumpTable[4*i+j]=1;
                  }
              }
              else if(idx_pred>dc && idx_pred<= dc+rc){
                  gait_jumpTable[4*i+0]=0; 
                  gait_jumpTable[4*i+1]=0;
                  gait_jumpTable[4*i+2]=1; 
                  gait_jumpTable[4*i+3]=1;
              }
              else{
                  for (int j=0; j<4; j++){
                      gait_jumpTable[4*i+j]=0;
                  }

              }
            
            }

    }

// ---------------------------------------------------------------------------
    double pf2com_ref_init[4] ={ r[0],
                         r[8],
                         r[2],
                         r[10]};
    // compute pf2com_ref: 
    for (int i =0 ; i<4; i++)
        pf2comw_ref[i] = pf2com_ref_init[i];
    

    // body rotation in 2D: y axis go inside
    Mat2<double> R_2D; // Need to define Mat2 in cppTypes.h (similarly check Mat3)
    R_2D << 1.f, 0.f, // initialization
          0.f, 1.f;
    
    // reference foot position w.r.t hip in body frame
    Vec2<double> pf2b_ref[2];

    Vec2<double> pf2comw[2]; // foot2com in world frame

    
    for (int i=1; i< horizonLength; i++){

      if(iterationCounter + i*iterationsBetweenMPC <= N_TrajRef)
      {
        idx_pred = iterationCounter + i*iterationsBetweenMPC;
      }
      else{
        idx_pred = N_TrajRef;

      }

      pf2b_ref[0]={pfDes[0][idx_pred], // front x
                   pfDes[1][idx_pred]}; // front z

      pf2b_ref[1]={pfDes[2][idx_pred], // rear x
                   pfDes[3][idx_pred]}; // rear z
      R_2D << cos(QDes[2][idx_pred]), sin(QDes[2][idx_pred]),
              -sin(QDes[2][idx_pred]), cos(QDes[2][idx_pred]);

      for (int i=0; i<2; i++){
                pf2comw[i] = R_2D*(data._quadruped->getHipLocation_2D(i) + pf2b_ref[i]);
      }

      pf2comw_ref[4*i+0] = pf2comw[0][0];
      pf2comw_ref[4*i+1] = pf2comw[0][1];
      pf2comw_ref[4*i+2] = pf2comw[1][0];
      pf2comw_ref[4*i+3] = pf2comw[1][1];

    }


 ////// -------------- Update Neural Network Ouput Prediction ---------------------//////////////

    Eigen::Matrix<fpt, 7, 1> q_pf_ct[horizonLength], q_pf_fl[horizonLength];
    // Get q_pf from current to future horizon
    // q_pf[0] << p[0], p[2],seResult.rpy[1],pFoot[0][0],pFoot[0][2], pFoot[2][0],pFoot[2][2];
    for(int i = 0; i < 4; i++) {
        pFoot[i] = seResult.position + seResult.rBody.transpose() 
                  * (data._quadruped->getHipLocation(i) + data._legController->data[i].p);
        // pFoot[i] = data._legController->data[i].p;
          //std::cout << "pFoot" << i << "\n" << pFoot[i] << std::endl;
        }

    for (int i = 0; i< horizonLength; i++){
      q_pf_ct[i] << trajAll_Jump[12*i+3], trajAll_Jump[12*i+5]-0.14, trajAll_Jump[12*i+1]+0.1, pf2comw_ref[4*i+0], pf2comw_ref[4*i+1], pf2comw_ref[4*i+2], pf2comw_ref[4*i+3];
      q_pf_fl[i] << trajAll_Jump[12*i+3]-QDes[0][800], trajAll_Jump[12*i+5]-(QDes[1][800]+0.14), trajAll_Jump[12*i+1]-QDes[2][800], pf2comw_ref[4*i+0], pf2comw_ref[4*i+1], pf2comw_ref[4*i+2], pf2comw_ref[4*i+3];
    
    }

    // Need to offset w.r.t 
    
    // try {
          

            Timer tff;
            tff.start();

              // using a tensor list
              std::vector<torch::Tensor> tensor_vec_ct, tensor_vec_fl;

            // std::vector<torch::jit::IValue> inputs_tensor;

            for (int h=0; h< horizonLength; h++){
              torch::Tensor input_tensor_ct = torch::tensor({{q_pf_ct[h][0], q_pf_ct[h][1],q_pf_ct[h][2],q_pf_ct[h][3],q_pf_ct[h][4], q_pf_ct[h][5],q_pf_ct[h][6]}}, {torch::kFloat32});//.requires_grad_(true);
              // std::cout << "Chuong_input:" << std::endl << input_tensor << "\n";
              torch::Tensor input_tensor_fl = torch::tensor({{q_pf_fl[h][0], q_pf_fl[h][1],q_pf_fl[h][2],q_pf_fl[h][3],q_pf_fl[h][4], q_pf_fl[h][5],q_pf_fl[h][6]}}, {torch::kFloat32});//.requires_grad_(true);
              // std::cout << "Chuong_input:" << std::endl << input_tensor << "\n";
             
              
              
              tensor_vec_ct.push_back(input_tensor_ct);
              tensor_vec_fl.push_back(input_tensor_fl);
            }

            torch::TensorList tensor_list_ct{tensor_vec_ct};
            auto batch_of_tensors_ct = torch::cat(tensor_list_ct);
            torch::TensorList tensor_list_fl{tensor_vec_fl};
            auto batch_of_tensors_fl = torch::cat(tensor_list_fl);

            // std::cout << input_tensor.device() << std::endl;
            atOutputg = module_gnet.forward({batch_of_tensors_ct}).toTensor();
            atOutputf1 = module_f1net.forward({batch_of_tensors_ct}).toTensor();
            atOutputf2 = module_f2net.forward({batch_of_tensors_fl}).toTensor();
            // std::cout << "OUTPUT atOutputg" << std::endl << atOutputg << "\n";
            // std::cout << "OUTPUT atOutputf1" << std::endl << atOutputf1 << "\n";
            printf("forward computation time %f ms\n", tff.getMs());
            t_nn = tff.getMs();


            for (int h=0; h<horizonLength; h++){
              for (int i = 0; i < 3; ++i) // number of rows
                  {
                      for (int j = 0; j < 4; ++j) { // number of columns
                            g_NN[12*h+4*i+j] = atOutputg[h][i][j].item<float>();
                            // cout<< "g:" << g_NN[12*h+4*i+j] << std::endl;

                      }
                  }
            
              for (int i = 0; i < 3; ++i) // number of rows
              {
                      f1_NN[3*h+i] = atOutputf1[h][i][0].item<float>();
                      // cout<< "f1:" << f1_NN[3*h+i] << std::endl;
              }

              for (int i = 0; i < 3; ++i) // number of rows
              {
                      // f2_NN[3*h+i] = 0.f;
                      f2_NN[3*h+i] = atOutputf2[h][i][0].item<float>();
                      // cout<< "f2:" << f2_NN[3*h+i] << std::endl;
              }
            
            // printf("forward computation time %f ms\n", tff.getMs());
          }
          
                      
      // }
      // catch (std::exception &e)
      //     {
      //          std::cout << e.what() << std::endl;
      //     }



    Timer t1;
    t1.start();
    dtMPC = dt * iterationsBetweenMPC;
    dtMPC_fl = dt * iterationsBetweenMPC_flight;
    setup_problem(dtMPC, dtMPC_fl, horizonLength,0.5,400); // 350 if use jumpMPC
    update_solver_settings(1 /*_parameters->jcqp_max_iter*/,1e-7 /* _parameters->jcqp_rho */,
      1e-8 /*_parameters->jcqp_sigma*/, 1.5 /* _parameters->jcqp_alpha*/, 1 /*_parameters->jcqp_terminate*/,0 /* _parameters->use_jcqp*/);
    //t1.stopPrint("Setup MPC");
    //std::cout << t1.getMs() << std::endl;
    Timer t2;
    t2.start();
    //cout << "dtMPC: " << dtMPC << "\n";
    update_problem_data(iterationCounter,p, v, q, w, r, pitch, weights, trajAll_Jump, FrefAll_Jump, pf2comw_ref, alpha, gait_jumpTable, g_NN, f1_NN, f2_NN);
    
    //t2.stopPrint("Run MPC");
    // printf("MPC Solve time %f ms\n", t2.getMs());
    //std::cout << t2.getSeconds() << std::endl;
    for(int leg = 0; leg < 4; leg++)
    {

      // std::cout << "leg:" << leg << std::endl;
      Vec3<double> delta_f; // command force:  delta_F or smt else
      Vec3<double> f_mpc; // computed force from MPC in world frame
      for(int axis = 0; axis < 3; axis++){
        // f[axis] = get_solution(leg*3 + axis);// original
        f_mpc[axis] = get_solution(leg*3 + axis);// original, is positive
        delta_f[axis] = f_mpc[axis]+ Fjump_Initial[leg*3 + axis]; // Delta F
      }

      f_MPC[leg] =  f_mpc; // output from MPC in world frame
      f_MPC_b[leg] =    - seResult.rBody * f_mpc; // convert to body frame, will be send to robot
      Delta_f[leg] =   - seResult.rBody * delta_f; // convert to body frame, will be send to robot

      // std::cout << "deltaF_mpc: " << std::endl;
      // std::cout << f_ff[leg] << std::endl;

      //std::cout << "mpc solution" << leg << "\n" << seResult.rBody * f << std::endl;
      // Update for WBC
       // Fr_des[leg] = f;      
    }


    printf("MPC Solve time %f ms\n", t2.getMs());
    t_mpc = t2.getMs();
    // time_record << 0 << " " << t_mpc << " " << 0 << std::endl;
    // printf("MPC Solve time %f ms\n", t2.getMs());
    // printf("update time: %.3f\n", t1.getMs());
  }

}

void ConvexMPCLocomotion::run(ControlFSMData& data)
{
  bool omniMode = false;


//  auto* debugSphere = data.visualizationData->addSphere();
//  debugSphere->color = {1,1,1,0.5};
//  debugSphere->radius = 1;
 // data._legController->updateData();
 // data._stateEstimator->run();
  auto& seResult = data._stateEstimator->getResult();
  auto& stateCommand = data._desiredStateCommand;
  //std::cout << "in side mpc" << seResult.rBody << std::endl;;

  // pick gait
  Gait* gait = &trotting;
  if(gaitNumber == 1)
    gait = &bounding;
  else if(gaitNumber == 2)
    gait = &trotting;
  else if(gaitNumber == 3)
    gait = &walking;
  else if(gaitNumber == 4)
    gait = &pacing;
  else if(gaitNumber == 5)
    gait = &galloping;
  else if(gaitNumber == 6)
    gait = &pronking;
  current_gait = gaitNumber;
  // integrate position setpoint
  Vec3<double> v_des_robot(stateCommand->data.stateDes(6), stateCommand->data.stateDes(7),0);
  Vec3<double> v_des_world;
  //v_des_world = coordinateRotation(CoordinateAxis::Z, seResult.rpy[2]).transpose() * v_des_robot;
  //Vec3<double> v_robot = coordinateRotation(CoordinateAxis::Z, seResult.rpy[2])*seResult.vWorld;

  v_des_world = seResult.rBody.transpose() * v_des_robot;
  Vec3<double> v_robot = seResult.vWorld;

  world_position_desired[0] += dt * v_des_world[0];
  world_position_desired[1] += dt * v_des_world[1];
  world_position_desired[2] = 0.3;
  
 // printf("p_des \t%.6f\n", dt * v_des_world[0]);
 //Integral-esque pitch and roll compensation
  if(fabs(v_robot[0]) > .2)   //avoid dividing by zero
  {
    rpy_int[1] += dt*(stateCommand->data.stateDes[4] /*-hw_i->state_estimator->se_ground_pitch*/ - seResult.rpy[1])/v_robot[0];
  }
  if(fabs(v_robot[1]) > 0.1)
  {
    rpy_int[0] += dt*(stateCommand->data.stateDes[3] /*-hw_i->state_estimator->se_ground_pitch*/
        - seResult.rpy[0])/v_robot[1];
  }

  rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);
  rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
  rpy_comp[1] = v_robot[0] * rpy_int[1];
  rpy_comp[0] = v_robot[1] * rpy_int[0] * (gaitNumber!=6);  //turn off for pronking

  


  for(int i = 0; i < 4; i++) {
   pFoot[i] = seResult.position + seResult.rBody.transpose() 
            * (data._quadruped->getHipLocation(i) + data._legController->data[i].p);
   // pFoot[i] = data._legController->data[i].p;
    //std::cout << "pFoot" << i << "\n" << pFoot[i] << std::endl;
  }
  
  if(climb){
    bool allContact = true;
    for(int i = 0; i < 4; i++){
      if(lowState.footForce[i] < 3 ){
        allContact = false;
      }
    }
  if(iterationCounter % (iterationsBetweenMPC * horizonLength) == 0){
    for(int i = 0; i < 4; i++)
        {
          W.row(i) << 1, pFoot[i][0], pFoot[i][1];
          pz[i] = pFoot[i][2];
         // if(i != 0) W.row(i) << 0, pFoot[i][0], pFoot[i][1];
        }
      a = W.transpose() * W * (W.transpose()* W * W.transpose()*W).inverse()*  W.transpose() * pz;
      ground_pitch = acos(-1/sqrt(a[1]*a[1] + a[2]*a[2] +1)) - 3.14;
      // std::cout << "ground pitch: " << ground_pitch << std::endl;
    }
  }

  if(climb){
    if(fabs(v_robot[0]) > .4)   //avoid dividing by zero
  {
    rpy_int[1] += dt*(stateCommand->data.stateDes[4] - seResult.rpy[1])/v_robot[0];
  }
  if(fabs(v_robot[1]) > 0.1)
  {
    rpy_int[0] += dt*(stateCommand->data.stateDes[3]
        - seResult.rpy[0])/v_robot[1];
  }
  }

  // some first time initialization
  if(firstRun)
  {
    std::cout << "Run MPC" << std::endl;
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.position[2];

    Vec3<double> v_des_robot(0,0,0);  // connect to desired state command later
    Vec3<double> v_des_world(0,0,0);  // connect to desired state command later

    Vec3<double> v_robot = seResult.vWorld;
    pBody_des[0] = world_position_desired[0];
    pBody_des[1] = world_position_desired[1];
    pBody_des[2] = world_position_desired[2];
    vBody_des[0] = v_des_world[0];
    vBody_des[1] = v_des_world[1];
    vBody_des[2] = 0;

    pBody_RPY_des[0] = 0;
    pBody_RPY_des[1] = 0;
    pBody_RPY_des[2] = 0; // seResult.rpy[2];

    vBody_Ori_des[0] = 0;
    vBody_Ori_des[1] = 0;
    vBody_Ori_des[2] = 0; // set this for now

    for(int i = 0; i < 4; i++){
      footSwingTrajectories[i].setHeight(0.08);
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);
      //std::cout << "orig foot pos " << i << pFoot[i] << std::endl;
    }
    firstRun = false;
  }
  
  // Update For WBC
  // pBody_des[0] = world_position_desired[0];
  // pBody_des[1] = world_position_desired[1];
  // pBody_des[2] = 0.4;

  // vBody_des[0] = v_des_world[0];
  // vBody_des[1] = v_des_world[1];
  // vBody_des[2] = 0.;

  // pBody_RPY_des[0] = 0.;
  // pBody_RPY_des[1] = 0.; 
  // pBody_RPY_des[2] = 0.; // stateCommand->data.stateDes[5];

  // vBody_Ori_des[0] = 0.;
  // vBody_Ori_des[1] = 0.;
  // vBody_Ori_des[2] = 0.; // stateCommand->data.stateDes[11];

  contact_state = gait->getContactSubPhase();

 
   // std::cout << "foot force 0 :" << lowState.footForce[0] << std::endl;
   // std::cout << "contact 0 : " << contact_state[0] << std::endl;
  
  // foot placement
  swingTimes[0] = dtMPC * gait->_swing;
  swingTimes[1] = dtMPC * gait->_swing;
  swingTimes[2] = dtMPC * gait->_swing;
  swingTimes[3] = dtMPC * gait->_swing;

  //std::cout << "Swing Time: " << swingTimes[0] << std::endl; 

  //std::cout << "Swing Time" << swingTimes << std::endl;
  double side_sign[4] = {-1, 1, -1, 1};
  double interleave_y[4] = {-0.08, 0.08, 0.01, -0.01};
  double interleave_gain = -0.2;
  double v_abs = std::fabs(seResult.vBody[0]);
  for(int i = 0; i < 4; i++)
  {

    if(firstSwing[i]) {
      swingTimeRemaining[i] = swingTimes[i];
    } else {
      swingTimeRemaining[i] -= dt;
    }

  
    if(firstSwing[i]) {
      
      footSwingTrajectories[i].setHeight(.05);
      Vec3<double> offset(0, side_sign[i] * data._quadruped->hipLinkLength, 0);
      // simple heuristic function
      //std::cout << "swing time" << swingTimeRemaining[i] << std::endl;
      
      Vec3<double> pRobotFrame = (data._quadruped->getHipLocation(i) + offset);
      Vec3<double> pYawCorrected = coordinateRotation(CoordinateAxis::Z, -stateCommand->data.stateDes[11] * gait->_stance * dtMPC / 2) * pRobotFrame;
 
      Vec3<double> des_vel;
      
      des_vel[0] = stateCommand->data.stateDes(6);
      des_vel[1] = stateCommand->data.stateDes(7);
      des_vel[2] = stateCommand->data.stateDes(8);

      Vec3<double> Pf = seResult.position +
                       seResult.rBody.transpose() * (pYawCorrected
                       + des_vel * swingTimeRemaining[i]);

      //+ seResult.vWorld * swingTimeRemaining[i];

      double p_rel_max = 0.4;
      double pfx_rel = seResult.vWorld[0] * .5 * gait->_stance * dtMPC +
                      .03*(seResult.vWorld[0]-v_des_world[0]) +
                      (0.5*seResult.position[2]/9.81) * (seResult.vWorld[1]*stateCommand->data.stateDes[11]);
      double pfy_rel = seResult.vWorld[1] * .5 * gait->_stance * dtMPC +
                      .03*(seResult.vWorld[1]-v_des_world[1]) +
                      (0.5*seResult.position[2]/9.81) * (-seResult.vWorld[0]*stateCommand->data.stateDes[11]);
      pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
      pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
      Pf[0] +=  pfx_rel;
      Pf[1] +=  pfy_rel + interleave_y[i] * v_abs * interleave_gain;
      Pf[2] = -0.01;
      //Pf[2] = 0;// pFoot[i][2];
      // if(climb){
      // Pf[2] = pFoot[i][2];
      // //a[0] + a[1] * Pf[0] + a[2] * Pf[1] - 0.02;
      // }
      footSwingTrajectories[i].setFinalPosition(Pf);
    }
    

  }




  // calc gait
  gait->setIterations(iterationsBetweenMPC, iterationCounter);
  
  // load LCM leg swing gains
  Kp << 700, 0, 0,
      0, 700, 0,
      0, 0, 350;
  Kp_stance = 0*Kp;

  Kd << 15, 0, 0,
      0, 15, 0,
      0, 0, 15;
  Kd_stance << 10, 0, 0,
               0, 10, 0,
               0, 0, 10;
  // gait
  Vec4<double> contactStates = gait->getContactSubPhase();
  Vec4<double> swingStates = gait->getSwingSubPhase();
  int* mpcTable = gait->mpc_gait();

  updateMPCIfNeeded(mpcTable, data, omniMode);
  
  iterationCounter++;

//  StateEstimator* se = hw_i->state_estimator;
  // Vec4<double> se_contactState(0,0,0,0);
  // for(int i = 0; i < 4; i++){
  //   footSwingTrajectories[i].setHeight(0.1);
  //   footSwingTrajectories[i].setInitialPosition(pFoot[i]);
  //   footSwingTrajectories[i].setFinalPosition(pFoot[i]);
  // }
  data._stateEstimator->setContactPhase(contactStates);

  for(int foot = 0; foot < 4; foot++)
  {
  
    double contactState = contactStates(foot);
    double swingState = swingStates(foot);

    // std::cout << "contact" << foot << ": " << contactState << std::endl;
    if(swingState > 0) // foot is in swing
    {
      if(firstSwing[foot])
      {
        firstSwing[foot] = false;
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);   
        //footSwingTrajectories[foot].setHeight(0.1);
      }
 
      //std::cout << "swing" << foot << ": " << swingState << std::endl;
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);

       Vec3<double> pDesFootWorld = footSwingTrajectories[foot].getPosition().cast<double>();
       Vec3<double> vDesFootWorld = footSwingTrajectories[foot].getVelocity().cast<double>();
       Vec3<double> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHipLocation(foot);
       Vec3<double> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      // Vec3<double> pDesLeg = footSwingTrajectories[foot].getPosition();
      // Vec3<double> vDesLeg = footSwingTrajectories[foot].getVelocity();
      //cout << "Foot " << foot << " relative velocity desired: " << vDesLeg.transpose() << "\n";
        data._legController->commands[foot].feedforwardForce << 0, 0, 0;
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = Kp;
        data._legController->commands[foot].kdCartesian = Kd;
        //std::cout << "foot Des world" << foot << ": \n " << pDesFootWorld(2) << std::endl; 
        //std::cout << "foot Des " << foot << ": \n " << pDesLeg(2) << std::endl; 
        //singularity barrier
        //data._legController->commands[foot].tau[2] = 
        //  50*(data._legController->data[foot].q(2)<.1)*data._legController->data[foot].q(2);
       // std::cout << "contat " << foot << ": " << contactState << std::endl;
      if(climb){
        if(lowState.footForce[foot] > 10 && swingState>0.5){
          //std::cout << "force changed for leg " << foot << std::endl;
          data._legController->commands[foot].kpCartesian = Kp_stance;
          data._legController->commands[foot].kdCartesian = 0 * Kd;
          data._legController->commands[foot].feedforwardForce << 0, 0, -10;
          contactState = 0;
          firstSwing[foot] = true;
        }
      }
      // se_contactState[foot] = contactState;

      
    }
  
    else if(contactState > 0) // foot is in stance
    {
      firstSwing[foot] = true;
       footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
       Vec3<double> pDesFootWorld = footSwingTrajectories[foot].getPosition().cast<double>();
       Vec3<double> vDesFootWorld = footSwingTrajectories[foot].getVelocity().cast<double>();
       Vec3<double> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHipLocation(foot);
       Vec3<double> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      // Vec3<double> pDesLeg = footSwingTrajectories[foot].getPosition();
      // Vec3<double> vDesLeg = footSwingTrajectories[foot].getVelocity();
       //cout << "Foot " << foot << " relative velocity desired: " << vDesLeg.transpose() << "\n";
       //std::cout << "robot pos: \n" << seResult.position << std::endl;
       //foot_swing << pDesFootWorld[2] << " ";
         data._legController->commands[foot].pDes = pDesLeg;
         data._legController->commands[foot].vDes = vDesLeg;
         data._legController->commands[foot].kpCartesian = Kp_stance; // 0
         data._legController->commands[foot].kdCartesian = Kd_stance;
       
        data._legController->commands[foot].feedforwardForce = f_ff[foot];
       
    }
  
  // se->set_contact_state(se_contactState); todo removed
  
  //data._legController->updateCommand();

  }
}

void ConvexMPCLocomotion::updateMPCIfNeeded(int* mpcTable, ControlFSMData& data, bool omniMode) 
{
  //iterationsBetweenMPC = 30;
  // std::cout << "MPC iteration: ";
  // std::cout << iterationCounter % iterationsBetweenMPC << std::endl;
  //data._legController->updateData();
  if((iterationCounter % iterationsBetweenMPC) == 0)
  {
   // std::cout << "runMPC" << std::endl;
    auto seResult = data._stateEstimator->getResult();
    auto& stateCommand = data._desiredStateCommand;

    std::cout << stateCommand->data.stateDes(6) << std::endl;
    std::cout << stateCommand->data.stateDes(7) << std::endl;
    double* p = seResult.position.data();
    double* v = seResult.vWorld.data();
    double* w = seResult.omegaWorld.data();
    double* q = seResult.orientation.data();

    // float* pf = (float*) seResult.position.data();
    // float* vf = (float*) seResult.vWorld.data();
    // float* wf = (float*) seResult.omegaWorld.data();
    // float* qf = (float*) seResult.orientation.data();

    double r[12];
    for(int i = 0; i < 12; i++)
      r[i] = pFoot[i%4][i/4] - seResult.position[i/4];

    double Q[12] = {0.25, 0.5, 20, 1.0, 1.0, 15, 0, 0, 0.3, 0.2, 0.2, 0.2};
    double yaw = seResult.rpy[2];
    double* weights = Q;
    double alpha = 1e-6; // make setting eventually

    //printf("current posistion: %3.f %.3f %.3f\n", p[0], p[1], p[2]);

    if(alpha > 1e-4)
    {

      std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
      alpha = 1e-5;
    }
    Vec3<double> v_des_robot(stateCommand->data.stateDes[6], stateCommand->data.stateDes[7],0);
    //Vec3<double> v_des_world = coordinateRotation(CoordinateAxis::Z, seResult.rpy[2]).transpose() * v_des_robot;

    Vec3<double> v_des_world = seResult.rBody.transpose() * v_des_robot;
    //float trajInitial[12] = {0,0,0, 0,0,.25, 0,0,0,0,0,0};

    
      const double max_pos_error = .1;
      double xStart = world_position_desired[0];
      double yStart = world_position_desired[1];
      //std::cout << "orig " << xStart << "  " << yStart << std::endl;
      //printf("orig \t%.6f\t%.6f\n", xStart, yStart);
      //printf("ref: \t%.6f\t%.6f\n", p[0], p[1]);

      if(xStart - p[0] > max_pos_error) xStart = p[0] + max_pos_error;
      if(p[0] - xStart > max_pos_error) xStart = p[0] - max_pos_error;

      if(yStart - p[1] > max_pos_error) yStart = p[1] + max_pos_error;
      if(p[1] - yStart > max_pos_error) yStart = p[1] - max_pos_error;

      world_position_desired[0] = xStart;
      world_position_desired[1] = yStart;

      //printf("xys: \t%.3f\t%3.f\n", xStart, yStart);
      //printf("perr \t%.3f\t%.3f\n", p[0] - world_position_desired[0], p[1] - world_position_desired[1]);
      
      double trajInitial[12] = {rpy_comp[0], //+ stateCommand->data.stateDes[3],  // 0
                                rpy_comp[1], //+ stateCommand->data.stateDes[4],    // 1
                                stateCommand->data.stateDes[5],    // 2
                                xStart,                                   // 3
                                yStart,                                   // 4
                                0.3 ,   // 5
                                0,                                        // 6
                                0,                                        // 7
                                stateCommand->data.stateDes[11],  // 8
                                v_des_world[0],                           // 9
                                v_des_world[1],                           // 10
                                0};                                       // 11
      // if(climb){
      //   trajInitial[1] = ground_pitch; 
      //   trajInitial[5] += a[0] + a[1] * xStart + a[2] * yStart;
      // }
     
       for(int i = 0; i < horizonLength; i++)
      {
        for(int j = 0; j < 12; j++)
          trajAll[12*i+j] = trajInitial[j];

        if(i == 0) // start at current position  TODO consider not doing this
        {
          //trajAll[3] = hw_i->state_estimator->se_pBody[0];
          //trajAll[4] = hw_i->state_estimator->se_pBody[1];
          //trajAll[2] = seResult.rpy[2];
        }
        else
        {
          trajAll[12*i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * v_des_world[0];
          trajAll[12*i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * v_des_world[1];
          trajAll[12*i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * stateCommand->data.stateDes[11];
          //std::cout << "yaw traj" <<  trajAll[12*i + 2] << std::endl;
        }
      }
    //}

        // for(int i = 0; i < 12; i++)
        //     printf("%.4f, ", trajAll[i]);

        // printf("\n\n");



    Timer t1;
    t1.start();
    dtMPC = dt * iterationsBetweenMPC;
    setup_problem(dtMPC,dtMPC_fl,horizonLength, 0.2,450);
    update_solver_settings(1 /*_parameters->jcqp_max_iter*/,1e-7 /* _parameters->jcqp_rho */,
      1e-8 /*_parameters->jcqp_sigma*/, 1.5 /* _parameters->jcqp_alpha*/, 1 /*_parameters->jcqp_terminate*/,0 /* _parameters->use_jcqp*/);
    //t1.stopPrint("Setup MPC");
    //std::cout << t1.getMs() << std::endl;
    Timer t2;
    t2.start();
    //cout << "dtMPC: " << dtMPC << "\n";
    // update_problem_data(p, v, q, w, r, yaw, weights, trajAll, alpha, mpcTable);
    
    //t2.stopPrint("Run MPC");
    // printf("MPC Solve time %f ms\n", t2.getMs());
    //std::cout << t2.getSeconds() << std::endl;
    for(int leg = 0; leg < 4; leg++)
    {
      Vec3<double> f;
      for(int axis = 0; axis < 3; axis++)
        f[axis] = get_solution(leg*3 + axis);
     
      //f_ff[leg] =  - coordinateRotation(CoordinateAxis::Z, seResult.rpy[2]) * f;
      f_ff[leg] =  - seResult.rBody * f;
      
      //std::cout << "leg: " << leg << std::endl;
      //std::cout << f_ff[leg] << std::endl;
      //std::cout << "mpc solution" << leg << "\n" << seResult.rBody * f << std::endl;
      // Update for WBC
       // Fr_des[leg] = f;      
    }

    // printf("update time: %.3f\n", t1.getMs());
  }



}