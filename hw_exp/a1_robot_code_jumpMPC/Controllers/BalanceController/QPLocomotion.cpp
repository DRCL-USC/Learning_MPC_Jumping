#include <iostream>
#include "QPLocomotion.h"
#include "../include/Utilities/Timer.h"
#include "../include/Math/orientation_tools.h"

using namespace ori;

/* =========================== Controller ============================*/
QPLocomotion::QPLocomotion()
{
  // std::cout << "init QP loco" << std::endl;
  rpy_comp[0] = 0;
  rpy_comp[1] = 0;
  rpy_comp[2] = 0;
  rpy_int[0] = 0;
  rpy_int[1] = 0;
  rpy_int[2] = 0;
  Kp.setZero();
  Kd.setZero();
  Kp_stance.setZero();
  Kd_stance.setZero();
  for(int i = 0; i < 4; i++){
    firstSwing[i] = true;
  }
}

void QPLocomotion::run(ControlFSMData& data)
{
  auto& seResult = data._stateEstimator->getResult();
  auto& stateCommand = data._desiredStateCommand;
  for(int i = 0; i < 4; i++){
  pFoot[i] = seResult.position +
          seResult.rBody.transpose() * (data._quadruped->getHipLocation(i) + data._legController->data[i].p);
  pFoot_act[i] = pFoot[i];
  }

   if(climb){
    // bool allContact = true;
    // for(int i = 0; i < 4; i++){
    //   if(lowState.footForce[i] < 3 ){
    //     allContact = false;
    //   }
    // }
    int cycle = (int) data._gaitScheduler->gaitData.periodTimeNominal/dt;
  if(iterationCounter % cycle == 0){
    for(int i = 0; i < 4; i++)
        {
          W.row(i) << 1, pFoot[i][0], pFoot[i][1];
          pz[i] = pFoot[i][2];
          if(i != 0) W.row(i) << 0, pFoot[i][0], pFoot[i][1];
        }
      a = W.transpose() * W * (W.transpose()* W * W.transpose()*W).inverse()*  W.transpose() * pz;
      ground_pitch = acos(-1/sqrt(a[1]*a[1] + a[2]*a[2] +1)) - 3.14;
      std::cout << "ground pitch: " << ground_pitch << std::endl;
    }
  }
  

  if(firstRun){
    // std::cout << "QP first run" << std::endl;
    //data._gaitScheduler->gaitData._currentGait = GaitType::TROT;
    data._gaitScheduler->gaitData._nextGait = GaitType::STATIC_WALK;
    // data._gaitScheduler->gaitData.zero();
    // data._gaitScheduler->initialize();
    // data._gaitScheduler->gaitData.periodTimeNominal = GaitCycleTime;
    // data._gaitScheduler->createGait();
    // data._gaitScheduler->createGait();

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
    pBody_RPY_des[2] = seResult.rpy[2];

    vBody_Ori_des[0] = 0;
    vBody_Ori_des[1] = 0;
    vBody_Ori_des[2] = 0; // set this for now

    for(int i = 0; i < 4; i++){
      footSwingTrajectories[i].setHeight(0.05);
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);
    }
    firstRun = false;
  }

  Vec3<double> v_des_robot(stateCommand->data.stateDes[6], stateCommand->data.stateDes[7],0);
  Vec3<double> v_des_world; // = coordinateRotation(CoordinateAxis::Z, seResult.rpy[2]).transpose() * v_des_robot;
  v_des_world = seResult.rBody.transpose() * v_des_robot;
  Vec3<double> v_robot = seResult.vWorld;
  data._gaitScheduler->step();


  Kp << 700, 0, 0,
        0, 700, 0,
        0, 0, 300;
 
  Kp_stance =  0 * Kp;

  Kd << 15, 0, 0,
        0, 15, 0,
        0, 0, 15;
  Kd_stance << 10, 0, 0,
               0, 10, 0,
               0, 0, 10;
  // for debug
  // kp.setZero();
  // kd.setZero();

 
  // calc gait
  //gait->setIterations(iterationsPerCyc, iterationCounter);
  //iterationCounter++;

  Vec4<double> contactStates;
  Vec4<double> swingStates;
 

  for(int i = 0; i < 4; i++){
    contactStates(i) = data._gaitScheduler->gaitData.contactStateScheduled(i);
    
    swingStates(i) = data._gaitScheduler->gaitData.phaseSwing(i);
    swingTimes[i] = data._gaitScheduler->gaitData.timeSwing(i);
  }

   double side_sign[4] = {-1, 1, -1, 1};
  double interleave_y[4] = {-0.08, 0.08, 0.01, -0.01};
  double interleave_gain = -0.3;
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
      Vec3<double> offset(0, side_sign[i] * .0838, 0);
      // simple heuristic function
      
      
      Vec3<double> pRobotFrame = (data._quadruped->getHipLocation(i) + offset);
      Vec3<double> pYawCorrected = coordinateRotation(CoordinateAxis::Z, -stateCommand->data.stateDes[11] * data._gaitScheduler->gaitData.timeStance[i] / 2) * pRobotFrame;
 
      Vec3<double> des_vel;
      des_vel[0] = stateCommand->data.stateDes(6);
      des_vel[1] = stateCommand->data.stateDes(7);
      des_vel[2] = 0;
      Vec3<double> Pf = seResult.position +
                       seResult.rBody.transpose() * (pYawCorrected
                       + des_vel * swingTimeRemaining[i]);

      //+ seResult.vWorld * swingTimeRemaining[i];

      double p_rel_max = 0.3;
     
      //Pf[2] = -0.01;
      Pf[2] = -0.01;
      if(climb){
      //Pf[2] = pFoot[i][2];
      // Pf[2] = a[0] + a[1] * Pf[0] + a[2] * Pf[1];
      }
   
      footSwingTrajectories[i].setFinalPosition(Pf);
    }
    

  }

  // std::cout << "contact \n" << contactStates << std::endl;
  // std::cout << "swing \n" << swingStates << std::endl;
  // Vec4<double> swingStates = gait->getSwingState();
 
  updateQP(data);
  Vec4<double> se_contactState(0,0,0,0);
  
  for(int foot = 0; foot < 4; foot++)
  {
    double swingState = swingStates(foot);
    int contactState = contactStates(foot);
    
    // std::cout << "swingState " << foot << ": " << swingState << std::endl;
    se_contactState(foot) = data._gaitScheduler->gaitData.phaseStance(foot);
    if(contactState == 0){
       if(firstSwing[foot])
      {
        firstSwing[foot] = false;
          footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
         
         //footSwingTrajectories[foot].setHeight(0.1);
      }
    footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
    Vec3<double> pDesFootWorld = footSwingTrajectories[foot].getPosition();
    Vec3<double> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
    
    Vec3<double> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) 
        - data._quadruped->getHipLocation(foot);
    Vec3<double> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
    // std::cout << "pDesleg" << foot << ": \n" << pDesLeg << std::endl;
    // std::cout << "vDesleg" << foot << ": \n" << vDesLeg << std::endl;
    data._legController->commands[foot].feedforwardForce << 0, 0, 0;
    data._legController->commands[foot].pDes = pDesLeg;
    data._legController->commands[foot].vDes = vDesLeg;
    data._legController->commands[foot].kpCartesian = Kp;
    data._legController->commands[foot].kdCartesian = Kd;
    pFoot_des[foot] = pDesLeg;
    //firstSwing[foot] = false;
    }
    if(contactState == 1){
    footSwingTrajectories[foot].computeSwingTrajectoryBezier(0, swingTimes[foot]);
    Vec3<double> pDesFootWorld = footSwingTrajectories[foot].getPosition();
    Vec3<double> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
    //vDesFootWorld << 0, 0, 0;
    Vec3<double> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) 
        - data._quadruped->getHipLocation(foot);
    Vec3<double> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
    data._legController->commands[foot].kpCartesian = Kp_stance;
    data._legController->commands[foot].kdCartesian = Kd_stance;
    data._legController->commands[foot].pDes = pDesLeg;
    data._legController->commands[foot].vDes = vDesLeg;
    data._legController->commands[foot].feedforwardForce = f_ff[foot];
    pFoot_des[foot] = pDesLeg ;
    //std::cout << foot << " : " << f_ff[foot] << std::endl;
    firstSwing[foot] = true;
    }
  
  }

  data._stateEstimator->setContactPhase(se_contactState);
  
}

void QPLocomotion::updateQP(ControlFSMData& data){

  auto seResult = data._stateEstimator->getResult();
  auto& stateCommand = data._desiredStateCommand;

  double minForce = 5;
  double maxForce = 300;
  double contactStateScheduled[4];
  
  double minForces[4] = {minForce, minForce, minForce, minForce};
  double maxForces[4] = {maxForce, maxForce, maxForce, maxForce};

  double COM_weights[3] = {5, 5, 10};
  double Base_weights[3] = {50, 25, 10};
  double pFeet_des[12],pFeet_act[12], pFeet[12], p_act[3], v_act[3], O_err[3], rpy[3],v_des[3], p_des[3],
         omegaDes[3];
  double se_xfb[13];
  double kpCOM[3], kdCOM[3], kpBase[3], kdBase[3];
  double b_control[6];

  for(int i = 0; i < 4; i++){
    se_xfb[i] = seResult.orientation(i);
    contactStateScheduled[i] = data._gaitScheduler->gaitData.contactStateScheduled(i);
    pFeet_des[i*3] = pFoot_des[i][0];
    pFeet_des[i*3+1] = pFoot_des[i][1];
    pFeet_des[i*3+2] = pFoot_des[i][2];
    //std::cout << contactStateScheduled[i] << " ";
  } 
  
  for(int i = 0; i < 3; i++){
    p_act[i] = seResult.position(i);
    v_act[i] = seResult.vWorld(i);

    se_xfb[4 + i] = seResult.position(i);
    se_xfb[7 + i] = seResult.omegaBody(i);
    se_xfb[10 + i] = seResult.vWorld(i);

    kpCOM[i] = 20;   //_data->controlParameters->kpCOM(i);
    kdCOM[i] = 5;     //_data->controlParameters->kdCOM(i);
    kpBase[i] = 200;     //_data->controlParameters->kpBase(i);
    kdBase[i] = 10; //  _data->controlParameters->kdBase(i);
  }
  kpCOM[2] = 0;
 kpBase[0] = 450;
    kdBase[0] = 4;
      kpBase[1] = 300;
  kdBase[1] = 6;
  Vec3<double> v_des_robot(stateCommand->data.stateDes[6], stateCommand->data.stateDes[7],0);
  Vec3<double> v_des_world; // = coordinateRotation(CoordinateAxis::Z, seResult.rpy[2]).transpose() * v_des_robot;
   v_des_world = seResult.rBody.transpose() * v_des_robot;
  //Vec3<double> v_robot = seResult.rBody*seResult.vWorld;
  //kpCOM[2] = 120;
  
  p_des[0] = stateCommand->data.stateDes(0);
  p_des[1] = stateCommand->data.stateDes(1);
  p_des[2] = 0.3;

  v_des[0] = v_des_world[0];
  v_des[1] = v_des_world[1];
  v_des[2] = 0;

  //std::cout << "vy_act" << seResult.vWorld[1] << std::endl;
 
  rpy[0] = 0;
  rpy[1] = 0;
  rpy[2] = stateCommand->data.stateDes[5];

  if(climb){
    rpy[1] = ground_pitch;
    // p_des[2] = 0.3 + a[0] + a[1] * p_des[0] + a[2] * p_des[1];
    }

  omegaDes[0] = 0;
  omegaDes[1] = 0;
  omegaDes[2] = stateCommand->data.stateDes[11];

  //std::cout << "yaw_act " << seResult.rpy[2] << std::endl;
  //std::cout << "yaw_des " << rpy[2] << std::endl;

  Vec3<double> pFeet_world;
  Vec3<double> pFeetVecCOM;

  for (int leg = 0; leg < 4; leg++) {
    pFeet_world = seResult.position +  seResult.rBody.transpose() *
    (data._quadruped->getHipLocation(leg) + data._legController->data[leg].p);
    pFeetVecCOM = seResult.rBody.transpose() *
    (data._quadruped->getHipLocation(leg) + data._legController->data[leg].p);


    pFeet[leg * 3] = pFeetVecCOM[0];
    pFeet[leg * 3 + 1] = pFeetVecCOM[1];
    pFeet[leg * 3 + 2] = pFeetVecCOM[2];

    pFeet_act[leg*3] = pFeet_world[0];
    pFeet_act[leg*3+1] = pFeet_world[1];
    pFeet_act[leg*3+2] = pFeet_world[2];
  }

  //Timer t1;
  //t1.start();

  QP.set_alpha_control(0.001);
  QP.set_friction(0.2);
  //QP.set_mass(19);
  QP.set_mass(data._quadruped->mass); // for a1 robot
  QP.set_wrench_weights(COM_weights, Base_weights);
  QP.set_PDgains(kpCOM, kdCOM, kpBase, kdBase);
  QP.set_desiredTrajectoryData(rpy, p_des, omegaDes, v_des);
  QP.set_desired_swing_pos(pFeet_des);
  QP.set_actual_swing_pos(pFeet_act);
  QP.SetContactData(contactStateScheduled, minForces, maxForces);

  
  QP.updateProblemData(se_xfb, pFeet, p_des, p_act, v_des, v_act,
                      O_err, seResult.rpy(2));

  double fOpt[12];
  QP.solveQP_nonThreaded(fOpt);
  
  //std::cout << "solve time" << t1.getMs() << std::endl;

  for(int leg = 0; leg < 4; leg++){
    f_ff[leg] << fOpt[leg * 3], fOpt[leg * 3 + 1], fOpt[leg * 3 + 2];
 //data._legController->commands[leg].feedforwardForce = f_ff[leg];
  //  std::cout << "leg " << leg << " : " << f_ff[2] << std::endl;
  }
  
}