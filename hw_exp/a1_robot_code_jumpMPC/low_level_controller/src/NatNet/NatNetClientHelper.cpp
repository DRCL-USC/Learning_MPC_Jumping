#include "../../include/NatNet/NatNetClientHelper.h"

std::vector<RigidBodyPose> ProcessFrameData(sFrameOfMocapData* data) {
    std::vector<RigidBodyPose> poses;
    for (int i = 0; i < data->nRigidBodies; ++i) {
        sRigidBodyData rbData = data->RigidBodies[i];
        if (rbData.params & 0x01) { // Check if tracking is valid
            RigidBodyPose pose;
            pose.id = rbData.ID;
            pose.x = rbData.x;
            pose.y = rbData.y;
            pose.z = rbData.z;
            pose.qx = rbData.qx;
            pose.qy = rbData.qy;
            pose.qz = rbData.qz;
            pose.qw = rbData.qw;
            poses.push_back(pose);
        }
    }
    return poses;
}

void DataHandler(sFrameOfMocapData* data, void* pUserData) {
    std::vector<RigidBodyPose>* pPoses = static_cast<std::vector<RigidBodyPose>*>(pUserData);
    *pPoses = ProcessFrameData(data);
}
