#include "../include/body.h"
#include "../include/LegController.h"
#include "../include/FootSwingTrajectory.h"
#include <iostream>
#include <stdio.h>
#include <fstream>

using namespace a1_robot;
using namespace std;

class Custom
{
    public:
    Custom(): control(LeggedType::A1, LOWLEVEL){
        quad.setQuadruped(2);
        control.InitCmdData(lowCmd);
        dphase = 1.0/swingTime;
        swing.open("swing.txt");
    }
    ~Custom(){delete legController;}

    Quadruped quad;
    LegController* legController = new LegController(quad);
    FootSwingTrajectory<double> footSwingTraj;
    Control control;

    double phase = 0;
    double swingTime = 250; // in ms
    double dphase;
    ofstream swing;

    bool firstswing = 1;

    void setTraj();
    void robotControl();
};

void Custom::setTraj(){
    legController->commands[0].kpCartesian << 1200, 0, 0,
                                              0, 1200, 0,
                                              0, 0, 300;

    legController->commands[0].kdCartesian  << 20, 0 ,0,
                                               0, 20, 0,
                                               0, 0, 10;

    Vec3<double> footPos = legController->data[0].p;
    Vec3<double> footDes(0.1, -0.15, -0.25);

    footSwingTraj.setInitialPosition(footPos);
    footSwingTraj.setFinalPosition(footDes);
    footSwingTraj.setHeight(0.1);
}

void Custom::robotControl(){
    Recv();
    legController->updateData();
    if(motiontime <= 1000){
    legController->commands[0].kpCartesian << 500, 0, 0,
                                              0, 500, 0,
                                              0, 0, 350;

    legController->commands[0].kdCartesian  << 10, 0 ,0,
                                               0, 10, 0,
                                               0, 0, 11;
    legController->commands[0].pDes << 0, -0.0838 , -0.25;
    }
    if(motiontime > 1000){

    if(firstswing){
        setTraj();
        firstswing = false;
    
    }
    

    phase += dphase;
    if(phase  >= 1){
        phase = 1;
              legController->commands[0].kpCartesian << 0, 0, 0,
                                              0, 0, 0,
                                              0, 0, 0;

    legController->commands[0].kdCartesian  << 8, 0 ,0,
                                               0, 8, 0,
                                               0, 0, 8;
        //return;
    }

    std::cout << "phase : " << phase << std::endl;

    footSwingTraj.computeSwingTrajectoryBezier(phase, swingTime/1000.0);
    Vec3<double> pDes = footSwingTraj.getPosition();
    Vec3<double> vDes = footSwingTraj.getVelocity();
    // std::cout << "foot Des" << pDes << std::endl;
    // std::cout << "swing phase" << phase << std::endl;
    if(motiontime < 1250){
        swing << legController->data[0].p[2] << " " << pDes[2] << " " <<  legController->data[0].p[0] << " " << pDes[0] << " " << legController->data[0].p[1] << " " << pDes[1] << 
        " " << legController->data[0].v[0] << " " << vDes[0] << " " << legController->data[0].v[1] << " " << vDes[1] << " " << legController->data[0].v[2] << " " << vDes[2] << std::endl;
    }
    legController->commands[0].pDes = pDes;
    legController->commands[0].vDes = vDes;
    }

    legController->updateCommand();

    
    control.PowerProtect(lowCmd, lowState, 10);
}

int main(){
    std::cout << "foot swing test" << std::endl;
    std::cout << "HANG UP ROBOT and CLEAR SURROUNDINGS" << std::endl;
    std::cin.ignore();

    Custom custom;

    LoopFunc loop_control("control_loop", 0.001, boost::bind(&Custom::robotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     0.001, 3, boost::bind(&Comm::UDPSend, &a1_robot::comm));
    LoopFunc loop_udpRecv("udp_recv",     0.001, 3, boost::bind(&Comm::UDPRecv, &a1_robot::comm));
   
    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while(1){
        sleep(10);
    };

    return 0;
}