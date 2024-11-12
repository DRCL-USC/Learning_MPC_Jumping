/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "../include/body.h"

using namespace UNITREE_LEGGED_SDK;

namespace a1_robot{

LowCmd lowCmd;
LowState lowState;
Comm comm;
xRockerBtnDataStruct _gamepad;
OptiTrack optitrack;
int motiontime = 0;
double dt = 0.001;
float OptiTrack_data[7]={0};

// These parameters are only for reference.
// Actual patameters need to be debugged if you want to run on real robot.
void paramInit()
{
    for(int i=0; i<4; i++){
        lowCmd.motorCmd[i*3+0].mode = 0x0A;
        lowCmd.motorCmd[i*3+0].Kp = 0;//70;
        lowCmd.motorCmd[i*3+0].dq = 0;
        lowCmd.motorCmd[i*3+0].Kd = 0;//3;
        lowCmd.motorCmd[i*3+0].tau = 0;
        lowCmd.motorCmd[i*3+1].mode = 0x0A;
        lowCmd.motorCmd[i*3+1].Kp = 0;//180;
        lowCmd.motorCmd[i*3+1].dq = 0;
        lowCmd.motorCmd[i*3+1].Kd = 0;//8;
        lowCmd.motorCmd[i*3+1].tau= 0;
        lowCmd.motorCmd[i*3+2].mode = 0x0A;
        lowCmd.motorCmd[i*3+2].Kp = 0;//300;
        lowCmd.motorCmd[i*3+2].dq = 0;
        lowCmd.motorCmd[i*3+2].Kd = 0;//15;
        lowCmd.motorCmd[i*3+2].tau = 0;
    }
}

void Recv(){
    comm.UDPRecv();
    comm.udp.GetRecv(lowState);
    memcpy(&_gamepad, lowState.wirelessRemote, sizeof(xRockerBtnDataStruct));
    //lowState = comm.state;

}

void Send(){
    motiontime++;
    comm.UDPSend();
}


void sendServoCmd()
{
    comm.cmd = lowCmd;
    comm.udp.SetSend(comm.cmd);
    Send();
    //usleep(1000); // in microseconds
}


}

void Comm::UDPRecv(){
    udp.Recv();
    
}

void Comm::UDPSend(){
    udp.Send();
}

// void Comm::getCmd(){
    
//     udp.GetRecv(state);
//     udp.SetSend(cmd);
// }

OptiTrack::OptiTrack(){
    client = new NatNetClient();
}

OptiTrack::~OptiTrack(){
    if (client){
        client->Disconnect();
        delete client;
    }
}

void OptiTrack::Connect(const char* serverIPAddress, const char* localIPAddress) {
    sNatNetClientConnectParams connectParams;
    connectParams.connectionType = ConnectionType_Multicast;
    connectParams.serverAddress = serverIPAddress;
    connectParams.localAddress = localIPAddress;

    if (client->Connect(connectParams) != ErrorCode_OK) {
        std::cerr << "Unable to connect to server." << std::endl;
    }
}

void OptiTrack::Disconnect() {
    if (client) {
        client->Disconnect();
    }
}

#include <chrono>  // Include chrono at the top of your file

// ...

void OptiTrack::RetrieveOptiTrackData() {
    auto start = std::chrono::high_resolution_clock::now();  // Start timing

    client->SetFrameReceivedCallback(DataHandler, &latestData);
    // std::cout << "Running Optitrack block" << std::endl;

    if (latestData.empty()) {
        std::cout << "No data in latestData" << std::endl;
    } else {
        // Assuming we are interested in the first RigidBodyPose data
        if (!latestData.empty()) {
            const RigidBodyPose& firstPose = latestData.front();
            a1_robot::OptiTrack_data[0] = firstPose.x;
            a1_robot::OptiTrack_data[1] = firstPose.y;
            a1_robot::OptiTrack_data[2] = firstPose.z;
            a1_robot::OptiTrack_data[3] = firstPose.qw;
            a1_robot::OptiTrack_data[4] = firstPose.qx;
            a1_robot::OptiTrack_data[5] = firstPose.qy;
            a1_robot::OptiTrack_data[6] = firstPose.qz;
            std::cout << "optitrack data x is " << firstPose.x << std::endl;
        }
    }

    auto end = std::chrono::high_resolution_clock::now();  // End timing
    std::chrono::duration<double, std::milli> duration = end - start;

    // std::cout << "RetrieveOptiTrackData() took " << duration.count() << " ms." << std::endl;
}