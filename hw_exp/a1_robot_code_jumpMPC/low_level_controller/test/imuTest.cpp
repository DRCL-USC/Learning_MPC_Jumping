#include "../include/LegController.h"
#include "../include/ContactEstimator.h"
#include "../include/OrientationEstimator.h"
#include "../include/PositionVelocityEstimator.h"
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
        quad.setQuadruped(2);
        control.InitCmdData(lowCmd);
        stateEstimator->addEstimator<ContactEstimator>();
        stateEstimator->addEstimator<OrientationEstimator>();
        stateEstimator->addEstimator<LinearKFPositionVelocityEstimator>();
        imu.open("v.txt");
    }
    ~Custom()
    {
        delete legController;
        delete stateEstimator;
        //imu.close();
    }

    Quadruped quad;
    StateEstimate stateEstimate;
    LegController* legController = new LegController(quad);
    StateEstimatorContainer* stateEstimator = new StateEstimatorContainer(
                                                &lowState.imu, legController->data, &stateEstimate);
    Control control;
    ofstream imu;
    void robotControl();
    
};

void Custom::robotControl()
{
    motiontime++;
    Recv();
    Vec4<double> Contact(0.5, 0.5 ,0.5 , 0.5);
    stateEstimator->setContactPhase(Contact);
    legController->updateData();
    stateEstimator->run();
    
    imu << stateEstimator->getResult().vWorld(0) << " " <<  stateEstimator->getResult().vWorld(1) << 
    " " << stateEstimator->getResult().position(0) << " " << stateEstimator->getResult().position(1) << 
    " " << stateEstimator->getResult().aWorld(0) << " " << stateEstimator->getResult().aWorld(1) << std::endl;
    //std::cout << "vy: " << stateEstimator->getResult().vWorld(1) << std::endl;
    // std::cout << "roll: " << stateEstimator->getResult().rpy(1) << std::endl;
    // std::cout << "lowState: " << lowState.imu.rpy[1] << std::endl;
    std::cout << "z leg" << legController->data[0].p(2) << std::endl;
    std::cout << "z Est: " << stateEstimator->getResult().position(2) << std::endl;
    legController->updateCommand();
    sendServoCmd();
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