#ifndef NATNETCLIENTHELPER_H
#define NATNETCLIENTHELPER_H

#include <iostream>
#include <vector>
#include "NatNetTypes.h"
#include "NatNetCAPI.h"
#include "NatNetClient.h"

struct RigidBodyPose {
    int id;
    float x, y, z; // Position
    float qx, qy, qz, qw; // Orientation (quaternion)
};

// Callback function to receive frame data
void DataHandler(sFrameOfMocapData* data, void* pUserData);

// Function to process and extract pose data from a frame
std::vector<RigidBodyPose> ProcessFrameData(sFrameOfMocapData* data);

#endif // NATNETCLIENTHELPER_H
