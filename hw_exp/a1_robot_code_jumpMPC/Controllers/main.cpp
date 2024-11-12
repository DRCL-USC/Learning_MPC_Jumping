#include "FSM.h"
#include "../low_level_controller/include/ContactEstimator.h"
#include "../low_level_controller/include/OrientationEstimator.h"
#include "../low_level_controller/include/PositionVelocityEstimator.h"
#include "../low_level_controller/include/LegController.h"
#include "../low_level_controller/include/body.h"

using namespace a1_robot;
//using namespace UNITREE_LEGGED_SDK;

int main(){
    Quadruped quad;
    StateEstimate stateEstimate;
    
    quad.setQuadruped(2);
    LegController* legController = new LegController(quad);
    StateEstimatorContainer* stateEstimator = new StateEstimatorContainer(
                                            &lowState.imu, legController->data, &stateEstimate);

    stateEstimator->addEstimator<ContactEstimator>();
    stateEstimator->addEstimator<OrientationEstimator>();
    stateEstimator->addEstimator<LinearKFPositionVelocityEstimator>();
    //stateEstimator->setup();

    GaitScheduler<double>* gaitScheduler = new GaitScheduler<double>(dt);
    //gaitScheduler->initialize();

    DesiredStateCommand* desiredStateCommand = new DesiredStateCommand(&stateEstimate, dt);

    ControlFSMData* _controlData = new ControlFSMData;

    std::cout << "set controlData" << std::endl;
    _controlData->_quadruped = &quad;
    _controlData->_stateEstimator = stateEstimator;
    _controlData->_gaitScheduler = gaitScheduler;
    _controlData->_legController = legController;
    _controlData->_desiredStateCommand = desiredStateCommand;

    //Optitrack setup
    optitrack.Connect("192.168.123.88", "192.168.123.162");
    
    FSM_State* fsm = new FSM_State(_controlData);
    std::cout << "fsm init" << std::endl;
    LoopFunc loop_control("control_loop", 0.001,  boost::bind(&FSM_State::Jump_MPC, fsm));
    LoopFunc loop_udpSend("udp_send",     0.001, 3, boost::bind(&Comm::UDPSend, &a1_robot::comm));
    LoopFunc loop_udpRecv("udp_recv",     0.001, 3, boost::bind(&Comm::UDPRecv, &a1_robot::comm));
    LoopFunc loop_optitrack("optitrack_loop", 0.001, boost::bind(&OptiTrack::RetrieveOptiTrackData, &optitrack));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();
    loop_optitrack.start();

    while(1){
        +sleep(10);
    }

    return 0;
}
