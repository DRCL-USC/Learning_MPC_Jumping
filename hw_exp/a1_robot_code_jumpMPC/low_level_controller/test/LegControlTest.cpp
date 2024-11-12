#include "../include/LegController.h"
#include "../include/body.h"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdint.h>

using namespace a1_robot;
using namespace std;

class Custom
{
    public:
    Custom():  control(LeggedType::A1, LOWLEVEL){
        quad.setQuadruped(2);
        control.InitCmdData(lowCmd);

    }
    ~Custom(){delete legController;}
    Quadruped quad;
    LegController* legController = new LegController(quad);
    Control control;

    void setcmd();
    void robotControl();

};
void Custom::setcmd(){

        legController->commands[0].kpCartesian << 350, 0, 0,
                                                  0, 350, 0,
                                                  0, 0, 250;

        legController->commands[0].kdCartesian  << 11, 0 ,0,
                                                   0, 11, 0,
                                                   0, 0, 11;

        legController->commands[0].pDes << 0, -0.0838 , -0.3;
    

    //legController->commands[i].feedforwardForce << 0, 0, -0.1;
    
    

    // if(motiontime > 5000){
    //     legController->commands[0].pDes << 0.2, -0.0838, -0.2;
    // }

    // if(motiontime > 6000){
    //     legController->commands[0].pDes << 0.1, -0.0838, -0.1;
    // }
    // if(motiontime > 7000){
    //     legController->commands[0].pDes << 0, -0.0838 , -0.3;
    // }
}

void Custom::robotControl(){
    motiontime++;
    Recv();
    //lowState = comm.state;
    setcmd();
    legController->updateData();
    //for(int i = 0; i < 3; i++){
     std::cout << "act : " << legController->data[0].p << std::endl;
    //}
    
   
    // for(int i = 0; i <4; i++){
    // std::cout << "cmd " << legController->commands[i].tau << std::endl;
    // }
    legController->updateCommand();
    //sendServoCmd();
    //comm.cmd = lowCmd;
    //comm.udp.SetSend(comm.cmd);
    control.PowerProtect(lowCmd, lowState, 10);
}



int main(){
    std::cout << "set to LOW-level" << std::endl;
    std::cout << "Clear surroundings" << std::endl;
    std::cin.ignore();

    Custom custom;

    LoopFunc loop_control("control_loop", 0.001, boost::bind(&Custom::robotControl, &custom));
    
    LoopFunc loop_udpSend("udp_send",     0.001, 3, boost::bind(&Comm::UDPSend, &a1_robot::comm));
    LoopFunc loop_udpRecv("udp_recv",     0.001, 3, boost::bind(&Comm::UDPRecv, &a1_robot::comm));
    //LoopFunc loop_feedback("control_feedback", 0.001, boost::bind(&LegController::updateData, &legController));
    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();
    //loop_feedback.start();

    while(1){
        sleep(10);
    };

    return 0;

}