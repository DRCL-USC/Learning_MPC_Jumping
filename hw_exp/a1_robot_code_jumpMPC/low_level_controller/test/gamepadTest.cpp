#include "../include/body.h"
#include <iostream>
#include <stdio.h>
#include <fstream>

using namespace a1_robot;
using namespace std;

class Custom
{
    public:
    Custom(): control(LeggedType::A1, LOWLEVEL){
        gamepad.open("gamepad.txt");
        control.InitCmdData(lowCmd);
    }
    ~Custom(){gamepad.close();}

    Control control;
    ofstream gamepad;

    void robotControl();
};

void Custom::robotControl(){
    Recv();
    for(int i = 0; i < 40; i++){
        gamepad << lowState.wirelessRemote[i] << " ";
        std::cout << lowState.wirelessRemote[i] << " " ;
    }
    gamepad << "\n";

    Send();
}

int main(){
    std::cout << "Remote control test" << std::endl;
    std::cout << "Clear surroundings" << std::endl;
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