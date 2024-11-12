#include "../include/LegController.h"
#include <iostream>
#include <fstream>


// upper level of joint controller 
// send data to joint controller
using namespace a1_robot;

/*!
 * Zero leg command
 */ 
void LegControllerCommand::zero(){
    tau = Vec3<double>::Zero();
    qDes = Vec3<double>::Zero();
    qdDes = Vec3<double>::Zero();
    pDes = Vec3<double>::Zero();
    vDes = Vec3<double>::Zero();
    feedforwardForce = Vec3<double>::Zero();
    force_MPC = Vec3<double>::Zero();
    kpCartesian = Mat3<double>::Zero(); 
    kdCartesian = Mat3<double>::Zero();
    kpJoint = Mat3<double>::Zero();
    kdJoint = Mat3<double>::Zero();
}

/*!
 * Zero leg data
 */ 
void LegControllerData::zero(){
    q = Vec3<double>::Zero();
    qd = Vec3<double>::Zero();
    p = Vec3<double>::Zero();
    v = Vec3<double>::Zero();
    J = Mat3<double>::Zero();
    tau = Vec3<double>::Zero();
}


void LegController::zeroCommand(){
    for (int i = 0; i<4; i++){
        commands[i].zero();
    }
}

void LegController::updateData(){
    for (int leg = 0; leg < 4; leg++){
        for(int j = 0; j<3; j++){
            data[leg].q(j) = (double)lowState.motorState[leg*3+j].q;
            data[leg].qd(j) = (double)lowState.motorState[leg*3+j].dq;
            data[leg].tau(j) = (double)lowState.motorState[leg*3+j].tauEst;
        }

        computeLegJacobianAndPosition(_quadruped, data[leg].q,&(data[leg].J),&(data[leg].p),leg);

         // v
        data[leg].v = data[leg].J * data[leg].qd;
    }
    //std::cout << "data updated" << std::endl;
}

void LegController::updateCommand(){

    for (int i = 0; i <4; i++){
        //computeLegJacobianAndPosition(_quadruped, data[i].q,&(data[i].J),&(data[i].p),i);
        // tauFF
        //commands[i].tau = Vec3<double>::Zero();
        Vec3<double> legTorque = commands[i].tau;
        //std::cout << "commmand" << commands[i].tau << std::endl;
        // forceFF

        Vec3<double> footForce = commands[i].feedforwardForce;

        footForce +=
            commands[i].kpCartesian * (commands[i].pDes - data[i].p);
         
        footForce +=
            commands[i].kdCartesian * (commands[i].vDes - data[i].v);
       // std::cout << "leg: " << i << std::endl;
       // std::cout << footForce << std::endl;
        // torque
        legTorque += data[i].J.transpose() * footForce;
        //std::cout << data[i].J << std::endl;
        commands[i].tau = legTorque;
        for (int j = 0; j<3; j++){
            lowCmd.motorCmd[i*3+j].q = (float)commands[i].qDes(j);
            lowCmd.motorCmd[i*3+j].dq = (float)commands[i].qdDes(j);
            lowCmd.motorCmd[i*3+j].tau = (float)commands[i].tau(j);
            lowCmd.motorCmd[i*3+j].Kp = (float)commands[i].kpJoint(j,j);
            lowCmd.motorCmd[i*3+j].Kd = (float)commands[i].kdJoint(j,j);
        
        }
	commands[i].tau << 0, 0, 0;
    }
    sendServoCmd();
    //std::cout << "cmd sent" << std::endl;
   
}

void LegController::updateCommand_MDC(){

    double Kt = 0.118; // 4/34
    double torque_motor_max = 4;
    double max_js = 1700 * 2 * 3.14 / 60;
    double min_js = 940 * 2 * 3.14 / 60;
    double _voltage_max = 21.5;
    double _current_max = 59.99;
    double _gear_ratio = 8.5;
    double _joint_vel_limit = 21;
    double _joint_torque_max = 33.5;
    double _R_motor = 25 * Kt * Kt;
    double voltage[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // voltage for all joints
    double current[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // current for all joints
    double total_current = 0;
    double total_torque = 0;
    double total_power = 0 ;
    
    for (int i =0; i<4; i++){
        for (int j = 0; j < 3; j++)
        {
            voltage[i * 3 + j] = commands[i].tau(j) * _R_motor / (Kt * _gear_ratio) + data[i].qd[j] * _gear_ratio * Kt;
            // cout << "voltage:" << voltage[i * 3 + j] << std::endl;
            if (voltage[i * 3 + j] > _voltage_max)
            {
                commands[i].tau[j] = (_voltage_max - 1.0 * data[i].qd[j] * _gear_ratio * Kt) * (Kt * _gear_ratio / _R_motor);
            }
            if (voltage[i * 3 + j] < -1.0 * _voltage_max)
            {
                commands[i].tau[j] = (-1.0 * _voltage_max - data[i].qd[j] * _gear_ratio * Kt) * (Kt * _gear_ratio / _R_motor);
            }
        }

    }

    // Send command to motors
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            lowCmd.motorCmd[i * 3 + j].q = commands[i].qDes[j];
            lowCmd.motorCmd[i * 3 + j].dq = commands[i].qdDes[j];
            lowCmd.motorCmd[i * 3 + j].tau = commands[i].tau[j];
            lowCmd.motorCmd[i * 3 + j].Kp = commands[i].kpJoint(j, j);
            lowCmd.motorCmd[i * 3 + j].Kd= commands[i].kdJoint(j, j);
            // std::cout << "positionStiffness: " << lowCmd.motorCmd[i*3+j].positionStiffness << std::endl;
            // std::cout << "velocityStiffness: " << lowCmd.motorCmd[i*3+j].velocityStiffness << std::endl;
        }
        commands[i].tau << 0, 0, 0; // zero torque command to prevent interference
    }

    sendServoCmd();
    std::cout << "cmd sent" << std::endl;
   
}

void LegController::updateCommand_Jump3D_4base(){

    for (int i = 0; i <4; i++){
        //computeLegJacobianAndPosition(_quadruped, data[i].q,&(data[i].J),&(data[i].p),i);
        // tauFF
        //commands[i].tau = Vec3<double>::Zero();
        Vec3<double> legTorque = commands[i].tau;
        //std::cout << "commmand" << commands[i].tau << std::endl;
        // forceFF

        Vec3<double> footForce = commands[i].feedforwardForce;

        footForce +=
            commands[i].kpCartesian * (commands[i].pDes - data[i].p);
         
        footForce +=
            commands[i].kdCartesian * (commands[i].vDes - data[i].v);
       // std::cout << "leg: " << i << std::endl;
       // std::cout << footForce << std::endl;
        // torque
        legTorque += data[i].J.transpose() * footForce;
        //std::cout << data[i].J << std::endl;
        commands[i].tau = legTorque;
        for (int j = 0; j<3; j++){
            lowCmd.motorCmd[i*3+j].q = (float)commands[i].qDes(j);
            lowCmd.motorCmd[i*3+j].dq = (float)commands[i].qdDes(j);
            lowCmd.motorCmd[i*3+j].tau = (float)commands[i].tau(j);
            lowCmd.motorCmd[i*3+j].Kp = (float)commands[i].kpJoint(j,j);
            lowCmd.motorCmd[i*3+j].Kd = (float)commands[i].kdJoint(j,j);
        
        }
	commands[i].tau << 0, 0, 0;
    }
    sendServoCmd();
    //std::cout << "cmd sent" << std::endl;
   
}

void computeLegJacobianAndPosition(Quadruped& _quad, Vec3<double>& q, Mat3<double>* J,Vec3<double>* p, int leg)
{
    double l1 = 0.0838; // ab_ad
    double l2 = 0.2;
    double l3 = 0.2;

    int sideSign = 1; // 1 for Left legs; -1 for right legs
    if (leg == 0 || leg == 2){
        sideSign = -1;
    }

    double s1 = std::sin(q(0));
    double s2 = std::sin(q(1));
    double s3 = std::sin(q(2));

    double c1 = std::cos(q(0));
    double c2 = std::cos(q(1));
    double c3 = std::cos(q(2));

    double c23 =  c2 * c3 - s2 * s3;
    double s23 =  s2 * c3 + c2 * s3; // sin(2+3))
   
   if(J){
    J->operator()(0, 0) = 0;
    J->operator()(1, 0) = -sideSign * l1 * s1 + l2 * c2 * c1 + l3 * c23 * c1;
    J->operator()(2, 0) = sideSign * l1 * c1 + l2 * c2 * s1 + l3 * c23 * s1;
    J->operator()(0, 1) = -l3 * c23 - l2 * c2;
    J->operator()(1, 1) = -l2 * s2 * s1 - l3 * s23 * s1;
    J->operator()(2, 1) = l2 * s2 * c1 + l3 * s23 * c1;
    J->operator()(0, 2) = -l3 * c23;
    J->operator()(1, 2) = -l3 * s23 *s1;
    J->operator()(2, 2) = l3 * s23 * c1;   
   }

   if(p){
    p->operator()(0) = -l3 * s23 - l2 * s2;
    p->operator()(1) = l1 * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1;
    p->operator()(2) = l1 * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2;
   }
}

