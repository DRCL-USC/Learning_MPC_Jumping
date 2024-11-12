#include "RobotState.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>

using std::cout;
using std::endl;

void RobotState::set(flt* p_, flt* v_, flt* q_, flt* w_, flt* r_,flt pitch_)
{
    for(u8 i = 0; i < 3; i++)
    {
        this->p(i) = p_[i];
        this->v(i) = v_[i];
        this->w(i) = w_[i];
    }
    this->q.w() = q_[0];
    this->q.x() = q_[1];
    this->q.y() = q_[2];
    this->q.z() = q_[3];
    this->pitch = pitch_;

    //for(u8 i = 0; i < 12; i++)
    //    this->r_feet(i) = r[i];
    for(u8 rs = 0; rs < 3; rs++)
        for(u8 c = 0; c < 4; c++)
            this->r_feet(rs,c) = r_[rs*4 + c];

    R = this->q.toRotationMatrix();
    fpt yc = cos(pitch_);
    fpt ys = sin(pitch_);

    // R_yaw <<  yc,  -ys,   0,
    //          ys,  yc,   0,
    //            0,   0,   1;
    // R_pitch <<  1,  0,   tan(pitch_),
    //          0,  1,   0,
    //            0,   0,   1/cos(pitch_);
    
    R_pitch <<  1,  0,   0,
             0,  1,   0,
               0,   0,   1;

    Matrix<fpt,3,1> Id;
    //Id << .033f, 0.161f, 0.175f;
    Id << .0168f, 0.0565f, 0.064f;
    I_body.diagonal() = Id;

    //TODO: Consider normalizing quaternion??
}

void RobotState::print()
{
   cout<<"Robot State:"<<endl<<"Position\n"<<p.transpose()
       <<"\nVelocity\n"<<v.transpose()<<"\nAngular Veloctiy\n"
       <<w.transpose()<<"\nRotation\n"<<R<<"\nYaw Rotation\n"
       <<R_pitch<<"\nFoot Locations\n"<<r_feet<<"\nInertia\n"<<I_body<<endl;
}



