/*!
 * @file PositionVelocityEstimator.h
 * @brief compute body position/velocity in world/body frames
 */ 



#ifndef PROJECT_POSITIONVELOCITYESTIMATOR_H
#define PROJECT_POSITIONVELOCITYESTIMATOR_H
#include "StateEstimatorContainer.h"

class LinearKFPositionVelocityEstimator : public GenericEstimator{
  public:
  
    LinearKFPositionVelocityEstimator();
    virtual void run();
    virtual void setup();

  private:
    Eigen::Matrix<double, 18, 1> _xhat;
    Eigen::Matrix<double, 12, 1> _ps;
    Eigen::Matrix<double, 12, 1> _vs;
    Eigen::Matrix<double, 18, 18> _A;
    Eigen::Matrix<double, 18, 18> _Q0;
    Eigen::Matrix<double, 18, 18> _P;
    Eigen::Matrix<double, 28, 28> _R0;
    Eigen::Matrix<double, 18, 3> _B;
    Eigen::Matrix<double, 28, 18> _C;
};

// class CheaterPositionVelocityEstimator : public GenericEstimator{
//   public:
//     virtual void run();
//     virtual void setup() {};
// };

class TunedKFPositionVelocityEstimator : public GenericEstimator{
  public:

    TunedKFPositionVelocityEstimator();
    virtual void run();
    virtual void setup();

  private:
    Eigen::Matrix<double, 18, 1> _xhat;   // state, pos vel foot pos
    Eigen::Matrix<double, 28,  1> _y;      // y measurement
    Eigen::Matrix<double, 28,  1> _yhat;   // y prediection
    Eigen::Matrix<double, 18, 18> _A;
    Eigen::Matrix<double, 18,  3> _B;
    Eigen::Matrix<double, 28, 18> _C;

    Eigen::Matrix<double, 12, 1> _ps;
    Eigen::Matrix<double, 12, 1> _vs;
    // Covariance matrix
    Eigen::Matrix<double, 18, 18> _P;
    Eigen::Matrix<double, 18, 18> _Ppri;
    Eigen::Matrix<double, 18, 18> _Q;
    Eigen::Matrix<double, 28, 28> _R;
    Eigen::Matrix<double, 18, 18> _QInit;
    Eigen::Matrix<double, 28, 28> _RInit;
    Vec18<double> _Qdig;
    Mat3<double> _Cu;

};
#endif