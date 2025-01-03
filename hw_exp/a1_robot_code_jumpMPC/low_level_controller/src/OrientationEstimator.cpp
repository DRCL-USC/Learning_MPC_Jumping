#include "../include/OrientationEstimator.h"

/*!
 * Get quaternion, rotation matrix, angular velocity (body and world),
 * rpy, acceleration (world, body) from vector nav IMU
 */
void OrientationEstimator::run() {
  //std::cout << "orientation" << std::endl;
  this->_stateEstimatorData.result->orientation[0] =
      this->_stateEstimatorData.imuData->quaternion[0];
  this->_stateEstimatorData.result->orientation[1] =
      this->_stateEstimatorData.imuData->quaternion[1];
  this->_stateEstimatorData.result->orientation[2] =
      this->_stateEstimatorData.imuData->quaternion[2];
  this->_stateEstimatorData.result->orientation[3] =
      this->_stateEstimatorData.imuData->quaternion[3];

  this->_stateEstimatorData.result->rBody = ori::quaternionToRotationMatrix(
      this->_stateEstimatorData.result->orientation);

  this->_stateEstimatorData.result->omegaBody(0) =
      this->_stateEstimatorData.imuData->gyroscope[0];
  this->_stateEstimatorData.result->omegaBody(1) =
      this->_stateEstimatorData.imuData->gyroscope[1];
  this->_stateEstimatorData.result->omegaBody(2) =
      this->_stateEstimatorData.imuData->gyroscope[2];
      this->_stateEstimatorData.result->rpy =
    ori::quatToRPY(this->_stateEstimatorData.result->orientation);

  this->_stateEstimatorData.result->omegaWorld =
      this->_stateEstimatorData.result->rBody.transpose() *
      this->_stateEstimatorData.result->omegaBody;

  this->_stateEstimatorData.result->rpy =
      ori::quatToRPY(this->_stateEstimatorData.result->orientation);
      
  this->_stateEstimatorData.result->aBody(0) =
      this->_stateEstimatorData.imuData->accelerometer[0];
  this->_stateEstimatorData.result->aBody(1) =
      this->_stateEstimatorData.imuData->accelerometer[1];   
  this->_stateEstimatorData.result->aBody(2) =
      this->_stateEstimatorData.imuData->accelerometer[2];

  this->_stateEstimatorData.result->aWorld =
      this->_stateEstimatorData.result->rBody.transpose() *
      this->_stateEstimatorData.result->aBody;
}
