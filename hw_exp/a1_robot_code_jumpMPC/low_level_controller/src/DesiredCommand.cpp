#include "../include/DesiredCommand.h"

void DesiredStateData::zero(){
    pre_stateDes = Vec12<double>::Zero();
    stateDes = Vec12<double>::Zero();
    // stateDes(5) = stateEstimate->rpy(2);
    // stateTrajDes = Eigen::Matrix<double, 12, 10>::Zero();
}
void DesiredStateCommand::getRCcommand(){
  _rcCommand.start = (int) _gamepad.btn.components.start;
  _rcCommand.lx = -_gamepad.lx;
  _rcCommand.ly = _gamepad.ly;
  _rcCommand.rx = -_gamepad.rx;

  if(_rcCommand.start_pressed && _rcCommand.start == 1){
    return;
  }
  
  if(!_rcCommand.start_pressed && _rcCommand.start == 1){
    _rcCommand.start_pressed = true;
    // std::cout << "start pressed" << std::endl;
  }
  
  if(_rcCommand.start_pressed && _rcCommand.start == 0){
    _rcCommand.start_pressed = false;
  }

  if(data.mode == 1 &&  _rcCommand.start_pressed){
    data.mode = 2;
    // std::cout << "switch to walking" << std::endl;
    return;
  }

  if(data.mode == 2 &&  _rcCommand.start_pressed){
    data.mode = 1;
    // std::cout << "switch to standing" << std::endl;
    return;
  }
}

void DesiredStateCommand::convertToStateCommands(){
    // data.zero();
    getRCcommand();
    static bool b_firstVisit(true);
    if(b_firstVisit){
    data.zero();
    data.pre_stateDes(0) = stateEstimate->position(0);
    data.pre_stateDes(1) = stateEstimate->position(1);
    data.pre_stateDes(5) = stateEstimate->rpy(2);
    b_firstVisit = false;
    }

    double vxCommand = _rcCommand.ly;
    double vyCommand = _rcCommand.rx;
    double yawCmd = _rcCommand.lx;
    if(data.mode == 1){
    // forward linear speed
   
    // Vertial linear speed
    data.stateDes(8) = 0.0;

    // X position
    data.stateDes(0) = stateEstimate->position(0) + dt * data.stateDes(6);
    //data.stateDes(0) = data.pre_stateDes(0) + dt * data.stateDes(6);

    // Y position
    data.stateDes(1) = stateEstimate->position(1) + dt * data.stateDes(7);
    //data.stateDes(1) = data.pre_stateDes(1) + dt * data.stateDes(7);
  
    // Z position height
    data.stateDes(2) = 0.3;

    // Roll rate
    data.stateDes(9) = 0.0;

    // Pitch rate
    data.stateDes(10) = 0.0;
    }

    if(data.mode == 2){
    data.stateDes(6) = deadband(vxCommand, minVelX, maxVelX);
    // std::cout << "vx: " << data.stateDes(6);
    // lateral linear speed
    data.stateDes(7) = deadband(vyCommand, minVelY, maxVelY);
      // std::cout << "vx: " << data.stateDes(6) << std::endl;
    // std::cout << "vy: " << data.stateDes(7) << std::endl;
    // X position
    data.stateDes(0) = stateEstimate->position(0) + dt * data.stateDes(6);
    //data.stateDes(0) = data.pre_stateDes(0) + dt * data.stateDes(6);

    // Y position
    data.stateDes(1) = stateEstimate->position(1) + dt * data.stateDes(7);
    //data.stateDes(1) = data.pre_stateDes(1) + dt * data.stateDes(7);
  

    //Yaw turn rate
    data.stateDes(11) =
     deadband(yawCmd, minTurnRate, maxTurnRate);

    // Roll
    data.stateDes(3) = 0.0;

    // Pitch
    data.stateDes(4) = 0.0;
     // deadband(rightAnalogStick[1], minPitch, maxPitch);

    // Yaw
    //data.stateDes(5) = stateEstimate->rpy(2) + dt * data.stateDes(11);
    data.stateDes(5) = data.pre_stateDes(5) + dt * data.stateDes(11);
  if(data.stateDes(5) > 3.141  && stateEstimate->rpy(2) < 0 ){
      data.stateDes(5) -= 6.28;//stateEstimate->rpy(2);
    }


     if(data.stateDes(5)  < -3.141 && stateEstimate->rpy(2) > 0){
      data.stateDes(5) += 6.28; //stateEstimate->rpy(2);
    }

    data.pre_stateDes(5) = data.stateDes(5);
    }
    data.pre_stateDes = data.stateDes;

  
}

void DesiredStateCommand::setStateCommands(Vec3<double> v_des, double yaw_rate){
    data.stateDes(6) = v_des[0];
    data.stateDes(7) = v_des[1];
    data.stateDes(8) = 0;

    data.stateDes(9) = 0;
    data.stateDes(10) = 0;
    data.stateDes(11) = yaw_rate;

    data.stateDes(0) = stateEstimate->position(0) + dt * data.stateDes(6);
    data.stateDes(1) = stateEstimate->position(1) + dt * data.stateDes(7);
    //data.stateDes(5) = stateEstimate->rpy(2) + dt * data.stateDes(11);
    data.stateDes(5) = data.pre_stateDes(5) + dt * data.stateDes(11);

    if(data.stateDes(5) > 3.141  && stateEstimate->rpy(2) < 0 ){
      data.stateDes(5) -= 6.28;//stateEstimate->rpy(2);
    }


     if(data.stateDes(5)  < -3.141 && stateEstimate->rpy(2) > 0){
      data.stateDes(5) += 6.28; //stateEstimate->rpy(2);
    }

    data.pre_stateDes(5) = data.stateDes(5);
}

// void DesiredStateCommand::desiredStateTrajectory(int N, Vec10<double> dtVec){
//     A = Mat12<double>::Zero();
//     A(0, 0) = 1;
//     A(1, 1) = 1;
//     A(2, 2) = 1;
//     A(3, 3) = 1;
//     A(4, 4) = 1;
//     A(5, 5) = 1;
//     A(6, 6) = 1;
//     A(7, 7) = 1;
//     A(8, 8) = 1; 
//     A(9, 9) = 1;
//     A(10, 10) = 1;
//     A(11, 11) = 1;
//     data.stateTrajDes.col(0) = data.stateDes;

//     for (int k = 1; k < N; k++) {
//     A(0, 6) = dtVec(k - 1);
//     A(1, 7) = dtVec(k - 1);
//     A(2, 8) = dtVec(k - 1);
//     A(3, 9) = dtVec(k - 1);
//     A(4, 10) = dtVec(k - 1);
//     A(5, 11) = dtVec(k - 1);
//     data.stateTrajDes.col(k) = A * data.stateTrajDes.col(k - 1);
//     for (int i = 0; i < 12; i++) {
//       // std::cout << data.stateTrajDes(i, k) << " ";
//     }
//     // std::cout << std::endl;
//   }
// }

double DesiredStateCommand::deadband(double command, double minVal, double maxVal){
    return command * maxVal;
}