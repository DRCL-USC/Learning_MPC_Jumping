#include "SolverMPC.h"
#include "common_types.h"
#include "convexMPC_interface.h"
#include "RobotState.h"
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
//#include <unsupported/Eigen/MatrixFunctions>
// #include "../qpOASES/include/qpOASES.hpp"
#include <qpOASES.hpp>
#include <stdio.h>
#include <sys/time.h>
#include "../include/Utilities/Timer.h"
#include "../JCQP/QpProblem.h"

//#define K_PRINT_EVERYTHING
#define BIG_NUMBER 5e10
//big enough to act like infinity, small enough to avoid numerical weirdness.

RobotState rs;
using std::cout;
using std::endl;
using Eigen::Dynamic;

//qpOASES::real_t a;

Matrix<fpt,Dynamic,13> A_qp;
Matrix<fpt,Dynamic,Dynamic> B_qp;
Matrix<fpt,Dynamic,Dynamic> B_qp_res;
Matrix<fpt,Dynamic,Dynamic> T, T1, T2;
Matrix<fpt,Dynamic,Dynamic> Hqp_res, H1_res, H2_res;
Matrix<fpt,13,12> Bdt[10];
// Matrix<fpt,13,12> Bdt_jump[10];
Matrix<fpt,13,12> Bdt_res[10];
Matrix<fpt,13,13> Adt, Adt1, Adt2;
Matrix<fpt,25,25> ABc, ABc2, expmm, ABc_res, expm_res;
Matrix<fpt,Dynamic,Dynamic> W;
Matrix<fpt,Dynamic,1> X_d;
Matrix<fpt,Dynamic,1> F_d;
Matrix<fpt,Dynamic,1> Fnn1_res, Fnn2_res; // neural network output residual
Matrix<fpt,Dynamic,1> U_b;
Matrix<fpt,Dynamic,Dynamic> fmat;

Matrix<fpt,Dynamic,Dynamic> qH;
Matrix<fpt,Dynamic,1> qg;

Matrix<fpt,Dynamic,Dynamic> eye_12h;

qpOASES::real_t* H_qpoases;
qpOASES::real_t* g_qpoases;
qpOASES::real_t* A_qpoases;
qpOASES::real_t* lb_qpoases;
qpOASES::real_t* ub_qpoases;
qpOASES::real_t* q_soln;

qpOASES::real_t* H_red;
qpOASES::real_t* g_red;
qpOASES::real_t* A_red;
qpOASES::real_t* lb_red;
qpOASES::real_t* ub_red;
qpOASES::real_t* q_red;
u8 real_allocated = 0;


char var_elim[2000];
char con_elim[2000];

mfp* get_q_soln()
{
  return q_soln;
}

s8 near_zero(fpt a)
{
  return (a < 0.01 && a > -.01) ;
}

s8 near_one(fpt a)
{
  return near_zero(a-1);
}
void matrix_to_real(qpOASES::real_t* dst, Matrix<fpt,Dynamic,Dynamic> src, s16 rows, s16 cols)
{
  s32 a = 0;
  for(s16 r = 0; r < rows; r++)
  {
    for(s16 c = 0; c < cols; c++)
    {
      dst[a] = src(r,c);
      a++;
    }
  }
}


void c2qp_learn(Matrix<fpt,13,13> Ac, Matrix<fpt,13,12> Bc[10], Matrix<fpt, 13,12> Bc_res[10],fpt dt1, s16 horizon, fpt f1nn[30])
// void c2qp(Matrix<fpt,13,13> Ac, Matrix<fpt,13,12> Bc,fpt dt,s16 horizon)

// compute Aqp1, Aqp2, Bqp1
{
  // ABc.setZero();
  // ABc.block(0,0,13,13) = Ac;
  // ABc.block(0,13,13,12) = Bc[0];
  // ABc = dt1*ABc;
  // expmm = ABc.exp();
  // Adt = expmm.block(0,0,13,13); // we only need to get Adt once time --> move out of the loop for below

  // for (int h=0; h< horizon; h++){
  //     ABc.block(0,0,13,13) = Ac;
  //     ABc.block(0,13,13,12) = Bc[h];
  //     ABc = dt1*ABc;
  //     expmm = ABc.exp();
  //     // Adt = expmm.block(0,0,13,13);
  //     Bdt[h] = expmm.block(0,13,13,12);
  // }

  // ABc_res.setZero();

  // for (int h=0; h<horizon; h++){

  //     ABc_res.block(0,0,13,13) = Ac;
  //     ABc_res.block(0,13,13,12) = Bc_res[h];
  //     ABc_res = dt1*ABc_res;
  //     expm_res = ABc_res.exp();
  //     Bdt_res[h] = expm_res.block(0,13,13,12); // what happen if Bc_res=0?

  // }

  Matrix<fpt,13,13> eye_13;
  eye_13.setIdentity();
  Adt = Ac*dt1 + eye_13;

  for (int h=0; h<horizon; h++){
    Bdt[h] = Bc[h]*dt1;
    Bdt_res[h] = Bc_res[h]*dt1;
  }


#ifdef K_PRINT_EVERYTHING
  cout<<"Adt: \n"<<Adt<<"\nBdt:\n"<<Bdt<<endl;
#endif
  if(horizon > 19) {
    throw std::runtime_error("horizon is too long!");
  }

  Matrix<fpt,13,13> powerMats[20];
  powerMats[0].setIdentity();
  for(int i = 1; i < horizon+1; i++) {
    powerMats[i] = Adt * powerMats[i-1];
  }

  for(s16 r = 0; r < horizon; r++)
  {
    A_qp.block(13*r,0,13,13) = powerMats[r+1];//Adt.pow(r+1);
    for(s16 c = 0; c < horizon; c++)
    {
      if(r >= c)
      {
        s16 a_num = r-c;
        B_qp.block(13*r,12*c,13,12) = powerMats[a_num] /*Adt.pow(a_num)*/ * Bdt[c];
        B_qp_res.block(13*r,12*c,13,12) = powerMats[a_num] /*Adt.pow(a_num)*/ * Bdt_res[c];
        T.block(13*r, 13*c, 13, 13) = powerMats[a_num];
      }
    }
  }
  B_qp+= B_qp_res;
  double M0_inv = 0.1;
  double I0_inv = 1/0.056;
  Matrix<double, 3, 1> outputf1;
  for(s16 r=0; r < horizon; r++){
      for (int i = 0; i < 3; i++){
          outputf1(i,0) = f1nn[3*r+i];
          // cout << "outputf1:" << outputf1(i,0)<< std::endl;
      }
      Fnn1_res.block(13*r,0,13,1) << 0, 0, 0, 0, 0, 0, 0, I0_inv*outputf1(2,0), 0, M0_inv*outputf1(0,0), 0, M0_inv*outputf1(1,0), 0;

  }
  Fnn1_res = Fnn1_res * dt1;
  
  // for (s16 r=0; r< horizon; r++){
  //   for (int k=0; k< r+1; k++){
  //       H_res.block(13*r,0,13,1) = powerMats[k] * Fnn1_res.block(13*(r-k),0,13,1); 
  //   }

  // 

  Hqp_res = T.block(0, 0, 13*horizon, 13*horizon)* Fnn1_res; // should have better way?




#ifdef K_PRINT_EVERYTHING
  cout<<"AQP:\n"<<A_qp<<"\nBQP:\n"<<B_qp<<endl;
#endif
}

void c2qp_learn_hybrid(Matrix<fpt,13,13> Ac, Matrix<fpt,13,12> Bc[10], Matrix<fpt, 13,12> Bc_res[10],fpt dt1, fpt dt2, s16 horizon, s16 N1, fpt f1nn[30], fpt f2nn[30])
// void c2qp(Matrix<fpt,13,13> Ac, Matrix<fpt,13,12> Bc,fpt dt,s16 horizon)

// compute Aqp1, Aqp2
{
  // ABc.setZero();
  // ABc.block(0,0,13,13) = Ac;
  // ABc.block(0,13,13,12) = Bc[0];
  // ABc = dt1*ABc;
  // expmm = ABc.exp();
  // Adt1 = expmm.block(0,0,13,13); // we only need to get Adt once time --> move out of the loop for below

  // ABc2.setZero();
  // ABc2.block(0,0,13,13) = Ac;
  // ABc2.block(0,13,13,12) = Bc[0];
  // ABc2 = dt2*ABc2;
  // expmm = ABc2.exp();
  // Adt2 = expmm.block(0,0,13,13);

  // for (int h=0; h< horizon; h++){ // should it be horizon or N1?
  //     ABc.block(0,0,13,13) = Ac;
  //     ABc.block(0,13,13,12) = Bc[h];
  //     ABc = dt1*ABc;
  //     expmm = ABc.exp();
  //     // Adt = expmm.block(0,0,13,13);
  //     Bdt[h] = expmm.block(0,13,13,12);
  // }


  // ABc_res.setZero();

  // for (int h=0; h<horizon; h++){

  //     ABc_res.block(0,0,13,13) = Ac;
  //     ABc_res.block(0,13,13,12) = Bc_res[h];
  //     ABc_res = dt1*ABc_res;
  //     expm_res = ABc_res.exp();
  //     Bdt_res[h] = expm_res.block(0,13,13,12); // what happen if Bc_res=0?

  // }


  Matrix<fpt,13,13> eye_13;
  eye_13.setIdentity();
  Adt1 = Ac*dt1 + eye_13;
  Adt2 = Ac*dt2 + eye_13;

  for (int h=0; h<horizon; h++){
    Bdt[h] = Bc[h]*dt1;
    Bdt_res[h] = Bc_res[h]*dt1;
  }




#ifdef K_PRINT_EVERYTHING
  cout<<"Adt: \n"<<Adt<<"\nBdt:\n"<<Bdt<<endl;
#endif
  if(horizon > 19) {
    throw std::runtime_error("horizon is too long!");
  }
  int N2 = horizon - N1; // go from 1 to 9
  Matrix<fpt,13,13> powerMats1[15], powerMats2[15];
  powerMats1[0].setIdentity();
  for(int i = 1; i < N1+1; i++) { // Note: N1: from 9 to 1
    powerMats1[i] = Adt1 * powerMats1[i-1]; // Adt1 ^ i
  }
  powerMats2[0].setIdentity();
  for(int i =1; i<N2+1; i++){
    powerMats2[i] = Adt2 * powerMats2[i-1];
  }

  // // Compute S and S_res
  // for (s16 c=0; c<N1; c++){
  //   S.block(0,12*c,13,12) = powerMats1[N1-c-1]*Bdt[c];
  //   S_res.block(0,12*c,13,12) = powerMats1[N1-c-1]*Bdt_res[c];
  // }

  // Compute A_qp

  for(s16 r = 0; r < horizon; r++)
  {
    if (r< N1){
      A_qp.block(13*r,0,13,13) = powerMats1[r+1];//Adt.pow(r+1); // Aqp{1}
    }
    else{
      A_qp.block(13*r,0,13,13) = powerMats1[N1]*powerMats2[r-N1+1]; // A1^N1*Aqp{2}
    }
  }

  // Compute B_qp

  for(s16 r=0; r< horizon; r++){
    for(s16 c = 0; c < N1; c++)
    {
      if(r >= c && r<N1)
      {
        s16 a_num = r-c;
        B_qp.block(13*r,12*c,13,12) = powerMats1[a_num] /*Adt.pow(a_num)*/ * Bdt[c];
        B_qp_res.block(13*r,12*c,13,12) = powerMats1[a_num] /*Adt.pow(a_num)*/ * Bdt_res[c];
      }
      
      if(r>= N1){
        B_qp.block(13*r,12*c,13,12)=powerMats2[r-N1+1]*powerMats1[N1-c-1]*Bdt[c]; // it is Aqp{2}*S
        B_qp_res.block(13*r,12*c,13,12)=powerMats2[r-N1+1]*powerMats1[N1-c-1]*Bdt_res[c];
      }
    }

  }
  B_qp+= B_qp_res;

  // Compute T1 and T2

  for(s16 r=0; r<N1; r++){
    for(s16 c=0; c<N1; c++){
      if(r>=c){
        s16 a_num = r-c;
        T1.block(13*r, 13*c, 13, 13) = powerMats1[a_num];
      }
    }
  }

  for(s16 r=0; r<N2; r++){
    for(s16 c=0; c<N2; c++){
      if(r>=c){
        s16 a_num = r-c;
        T2.block(13*r, 13*c, 13, 13) = powerMats2[a_num];
      }
    }
  }

  // Compute H1_res and H2_res, then finally H_res

  double M0_inv = 0.1;
  double I0_inv = 1/0.056;
  Matrix<double, 3, 1> outputf1, outputf2;
  for(s16 r=0; r < N1; r++){
      for (int i = 0; i < 3; i++){
          outputf1(i,0) = f1nn[3*r+i]; //f1 is only used during contact phase 
          // cout << "outputf1:" << outputf1(i,0)<< std::endl;
      }
      Fnn1_res.block(13*r,0,13,1) << 0, 0, 0, 0, 0, 0, 0, I0_inv*outputf1(2,0), 0, M0_inv*outputf1(0,0), 0, M0_inv*outputf1(1,0), 0;
  }
  Fnn1_res = Fnn1_res * dt1;

  for(s16 r=0; r < N2; r++){
      for (int i = 0; i < 3; i++){
          // outputf2(i,0) = f2nn[3*(N1+r)+i];// only shift if it is full
          outputf2(i,0) = f2nn[3*(r)+i];// f2 is only used during flight phase --> we don't shift index
          // cout << "outputf1:" << outputf1(i,0)<< std::endl;
      }
      Fnn2_res.block(13*r,0,13,1) << 0, 0, 0, 0, 0, 0, 0, I0_inv*outputf2(2,0), 0, M0_inv*outputf2(0,0), 0, M0_inv*outputf2(1,0), 0;
  }
  Fnn2_res = Fnn2_res * dt2;

  H1_res = T1.block(0,0, 13*N1, 13*N1) * Fnn1_res.block(0,0,13*N1,1);
  H2_res = T2.block(0,0, 13*N2, 13*N2) * Fnn2_res.block(0,0,13*N2,1);

  // compute Phi_res

  Matrix<fpt, 13, 1> phi_res;
  phi_res.setZero();
  for (s16 r=0; r< N1; r++){
    phi_res += powerMats1[N1-r-1]*Fnn1_res.block(13*r,0,13,1);
  }

  // get H_res
  Hqp_res.block(0,0, 13*N1, 1) = H1_res;
  for (s16 r=0; r<N2; r++){
      Hqp_res.block(13*(N1+r),0, 13, 1) = H2_res.block(13*r, 0, 13, 1) + powerMats2[r+1]*phi_res;
  }


#ifdef K_PRINT_EVERYTHING
  cout<<"AQP:\n"<<A_qp<<"\nBQP:\n"<<B_qp<<endl;
#endif
}


void resize_qp_mats(s16 horizon)
{
  int mcount = 0;
  int h2 = horizon*horizon;

  A_qp.resize(13*horizon, Eigen::NoChange);
  mcount += 13*horizon*1;

  B_qp.resize(13*horizon, 12*horizon);
  mcount += 13*h2*12;

  B_qp_res.resize(13*horizon, 12*horizon);
  mcount += 13*h2*12;

  T.resize(13*horizon, 13*horizon);
  mcount += 13*h2*13;

  T1.resize(13*horizon, 13*horizon);
  mcount += 13*h2*13;

  T2.resize(13*horizon, 13*horizon);
  mcount += 13*h2*13;

  Hqp_res.resize(13*horizon, Eigen::NoChange);
  mcount += 13*horizon;

  H1_res.resize(13*horizon, Eigen::NoChange);
  mcount += 13*horizon;

  H2_res.resize(13*horizon, Eigen::NoChange);
  mcount += 13*horizon;


  W.resize(13*horizon, 13*horizon);
  mcount += 13*13*h2;

  X_d.resize(13*horizon, Eigen::NoChange);
  mcount += 13*horizon;

  Fnn1_res.resize(13*horizon, Eigen::NoChange);
  mcount += 13*horizon;

  Fnn2_res.resize(13*horizon, Eigen::NoChange);
  mcount += 13*horizon;

  F_d.resize(12*horizon, Eigen::NoChange);
  mcount += 12*horizon;

  U_b.resize(20*horizon, Eigen::NoChange);
  mcount += 20*horizon;

  fmat.resize(20*horizon, 12*horizon);
  mcount += 20*12*h2;

  qH.resize(12*horizon, 12*horizon);
  mcount += 12*12*h2;

  qg.resize(12*horizon, Eigen::NoChange);
  mcount += 12*horizon;

  eye_12h.resize(12*horizon, 12*horizon);
  mcount += 12*12*horizon;

  //printf("realloc'd %d floating point numbers.\n",mcount);
  mcount = 0;

  A_qp.setZero();
  B_qp.setZero(); B_qp_res.setZero();
  T.setZero(); T1.setZero(); T2.setZero();
  Hqp_res.setZero(); H1_res.setZero(); H2_res.setZero();
  W.setZero();
  X_d.setZero();
  Fnn1_res.setZero(); Fnn2_res.setZero();
  F_d.setZero();
  U_b.setZero();
  fmat.setZero();
  qH.setZero();
  qg.setZero();
  eye_12h.setIdentity();

  //TODO: use realloc instead of free/malloc on size changes

  if(real_allocated)
  {

    free(H_qpoases);
    free(g_qpoases);
    free(A_qpoases);
    free(lb_qpoases);
    free(ub_qpoases);
    free(q_soln);
    free(H_red);
    free(g_red);
    free(A_red);
    free(lb_red);
    free(ub_red);
    free(q_red);
  }

  H_qpoases = (qpOASES::real_t*)malloc(12*12*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*12*h2;
  g_qpoases = (qpOASES::real_t*)malloc(12*1*horizon*sizeof(qpOASES::real_t));
  mcount += 12*horizon;
  A_qpoases = (qpOASES::real_t*)malloc(12*20*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*20*h2;
  lb_qpoases = (qpOASES::real_t*)malloc(20*1*horizon*sizeof(qpOASES::real_t));
  mcount += 20*horizon;
  ub_qpoases = (qpOASES::real_t*)malloc(20*1*horizon*sizeof(qpOASES::real_t));
  mcount += 20*horizon;
  q_soln = (qpOASES::real_t*)malloc(12*horizon*sizeof(qpOASES::real_t));
  mcount += 12*horizon;

  H_red = (qpOASES::real_t*)malloc(12*12*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*12*h2;
  g_red = (qpOASES::real_t*)malloc(12*1*horizon*sizeof(qpOASES::real_t));
  mcount += 12*horizon;
  A_red = (qpOASES::real_t*)malloc(12*20*horizon*horizon*sizeof(qpOASES::real_t));
  mcount += 12*20*h2;
  lb_red = (qpOASES::real_t*)malloc(20*1*horizon*sizeof(qpOASES::real_t));
  mcount += 20*horizon;
  ub_red = (qpOASES::real_t*)malloc(20*1*horizon*sizeof(qpOASES::real_t));
  mcount += 20*horizon;
  q_red = (qpOASES::real_t*)malloc(12*horizon*sizeof(qpOASES::real_t));
  mcount += 12*horizon;
  real_allocated = 1;

  //printf("malloc'd %d floating point numbers.\n",mcount);



#ifdef K_DEBUG
  printf("RESIZED MATRICES FOR HORIZON: %d\n",horizon);
#endif
}

inline Matrix<fpt,3,3> cross_mat(Matrix<fpt,3,3> I_inv, Matrix<fpt,3,1> r)
{
  Matrix<fpt,3,3> cm;
  cm << 0.f, -r(2), r(1),
    r(2), 0.f, -r(0),
    -r(1), r(0), 0.f;
  return I_inv * cm;
}
//continuous time state space matrices.
void ct_ss_mats_jump(Matrix<fpt,3,3> R_pitch, Matrix<fpt,13,13>& A)
{
  A.setZero();
  A(3,9) = 1.f;
  A(4,10) = 1.f;
  A(5,11) = 1.f;
  A(11,12) = 1.f;
  A.block(0,6,3,3) = R_pitch.transpose();

  // B.setZero();
  // Matrix<fpt,3,3> I_inv = I_world.inverse();

  // for(s16 b = 0; b < 4; b++)
  // {
  //   B.block(6,b*3,3,3) = cross_mat(I_inv,r_feet.col(b));
  //   B.block(9,b*3,3,3) = Matrix<fpt,3,3>::Identity() / m;
  // }

}


// void ct_ss_mats(Matrix<fpt,3,3> I_world, fpt m, Matrix<fpt,3,4> r_feet, Matrix<fpt,3,3> R_pitch, Matrix<fpt,13,13>& A, Matrix<fpt,13,12>& B)
// {
//   A.setZero();
//   A(3,9) = 1.f;
//   A(4,10) = 1.f;
//   A(5,11) = 1.f;
//   A(11,12) = 1.f;
//   A.block(0,6,3,3) = R_pitch.transpose();

//   B.setZero();
//   Matrix<fpt,3,3> I_inv = I_world.inverse();

//   for(s16 b = 0; b < 4; b++)
//   {
//     B.block(6,b*3,3,3) = cross_mat(I_inv,r_feet.col(b));
//     B.block(9,b*3,3,3) = Matrix<fpt,3,3>::Identity() / m;
//   }
// }



void quat_to_rpy(Quaternionf q, Matrix<fpt,3,1>& rpy)
{
  //from my MATLAB implementation

  //edge case!
  fpt as = t_min(2.*(q.w()*q.y() - q.x()*q.z()),.99999);
  rpy(0) = atan2(2.f*(q.w()*q.x()+q.y()*q.z()),1. - 2.f * (sq(q.x()) + sq(q.y())));
  rpy(1) = asin(as);
  rpy(2) = atan2(2.f*(q.w()*q.z()+q.x()*q.y()),1. - 2.f * (sq(q.y()) + sq(q.z())));
  // std::cout << "MPC solver rpy: " << rpy(0) << " " << rpy(1) << " " << rpy(2) << std::endl;
}
void print_problem_setup(problem_setup* setup)
{
  printf("DT: %.3f\n",setup->dt);
  printf("Mu: %.3f\n",setup->mu);
  printf("F_Max: %.3f\n",setup->f_max);
  printf("Horizon: %d\n",setup->horizon);
}

void print_update_data(update_data_t* update, s16 horizon)
{
  print_named_array("p",update->p,1,3);
  print_named_array("v",update->v,1,3);
  print_named_array("q",update->q,1,4);
  print_named_array("w",update->r,3,4);
  pnv("Pitch",update->pitch);
  print_named_array("weights",update->weights,1,12);
  print_named_array("trajectory",update->traj,horizon,12);
  pnv("Alpha",update->alpha);
  print_named_array("gait",update->gait,horizon,4);
}


Matrix<fpt,13,1> x_0;
Matrix<fpt,3,3> I_world;
Matrix<fpt,13,13> A_ct;
Matrix<fpt,13,12> B_ct_r;
Matrix<fpt, 13,12> Bc[10];
Matrix<fpt, 3, 4> r_feet_com;



void solve_mpc(update_data_t* update, problem_setup* setup)
{
  rs.set(update->p, update->v, update->q, update->w, update->r, update->pitch);
#ifdef K_PRINT_EVERYTHING

  printf("-----------------\n");
    printf("   PROBLEM DATA  \n");
    printf("-----------------\n");
    print_problem_setup(setup);

    printf("-----------------\n");
    printf("    ROBOT DATA   \n");
    printf("-----------------\n");
    rs.print();
    print_update_data(update,setup->horizon);
#endif

  Matrix<fpt,13,12> Bc_res[setup->horizon]; // at current time step
  for (int h=0; h< setup->horizon; h++){
      Bc_res[h].setZero();
  }

  // Matrix<fpt,13,12> Bc_res_0; // at current time step
  // Bc_res_0.setZero();

  // Matrix<fpt, 13,1> Fc_res[setup->horizon];
  double M0_inv = 0.1;
  double I0_inv = 1/0.056;
  Matrix<double, 3, 4> outputg;
  // Matrix<double, 3, 1> outputf1;

  // update->r:  order: FRx, FLx, RRx, RLx, FRy, FLy, RRy, RLy, FRz, FLz, RRz, RLz 
  
  try{

        for (int h = 0; h< setup-> horizon; h++){

          // get NN_g at this horizon

                for (int i = 0; i < 3; ++i) 
                    {
                          for (int j = 0; j < 4; ++j) {
                                outputg(i, j) = update->outg_NN[12*h + 4*i + j];
                                // cout << "outputg:" << outputg(i, j)<< std::endl;
                          }
                          // outputf1(i,0) = update->outf1_NN[3*h+i];
                          // // cout << "outputf1:" << outputf1(i,0)<< std::endl;
                    }
                
                // compute matrix B_res
                Eigen::Matrix<fpt, 13, 12> Bc_res_h;
                Bc_res_h.setZero();
                Bc_res_h.block(7,0,1,12) << I0_inv*outputg(2,0), 0 , I0_inv*outputg(2,1), I0_inv*outputg(2, 0), 0, I0_inv*outputg(2,1), I0_inv*outputg(2,2), 0 , I0_inv*outputg(2,3), I0_inv*outputg(2, 2), 0, I0_inv*outputg(2,3);
                Bc_res_h.block(9,0,1,12) << M0_inv*outputg(0,0), 0 , M0_inv*outputg(0,1), M0_inv*outputg(0, 0), 0, M0_inv*outputg(0,1), M0_inv*outputg(0,2), 0 , M0_inv*outputg(0,3), M0_inv*outputg(0, 2), 0, M0_inv*outputg(0,3); 
                Bc_res_h.block(11,0,1,12) << M0_inv*outputg(1,0), 0 , M0_inv*outputg(1,1), M0_inv*outputg(1, 0), 0, M0_inv*outputg(1,1), M0_inv*outputg(1,2), 0 , M0_inv*outputg(1,3), M0_inv*outputg(1, 2), 0, M0_inv*outputg(1,3);
                                
                Bc_res[h] = Bc_res_h;

                // compute matrix Fnn_res
                // Eigen::Matrix<fpt, 13, 1> Fc_res_h;
                // Fc_res_h << 0, 0, 0, 0, 0, 0, 0, I0_inv*outputf1(2,0), 0, M0_inv*outputf1(0,0), 0, M0_inv*outputf1(1,0), 0;
                // for (int j=0; j<13; j++){
                //       Fnn_res(13*h+j,0) = Fc_res_h(j,0);
                // }
                // Fnn1_res.block(13*h,0,13,1) << 0, 0, 0, 0, 0, 0, 0, I0_inv*outputf1(2,0), 0, M0_inv*outputf1(0,0), 0, M0_inv*outputf1(1,0), 0;
            }
  }

  catch (std::exception &e)
  {
        std::cout << e.what() << std::endl;
  }

  //roll pitch yaw
  Matrix<fpt,3,1> rpy;
  quat_to_rpy(rs.q,rpy);

  //initial state (13 state representation)
  x_0 << rpy(0), rpy(1), rpy(2), rs.p , rs.w, rs.v, -9.8f;
  I_world = rs.R_pitch * rs.I_body * rs.R_pitch.transpose(); //original
  // I_world = rs.R_pitch * rs.I_body * rs.R_pitch.transpose(); //chuong
  //I_world = rs.R_yaw.transpose() * rs.I_body * rs.R_yaw;
  cout<< "R_pitch in solverMPC.cpp:" << rs.R_pitch<<endl;

  // compute A, B in continuous space
  // ct_ss_mats(I_world,rs.m,rs.r_feet,rs.R_pitch,A_ct,B_ct_r);
  ct_ss_mats_jump(rs.R_pitch,A_ct); // we compute nominal Bc outside of this function as below


#ifdef K_PRINT_EVERYTHING
  cout<<"Initial state: \n"<<x_0<<endl;
    cout<<"World Inertia: \n"<<I_world<<endl;
    cout<<"A CT: \n"<<A_ct<<endl;
    cout<<"B CT (simplified): \n"<<B_ct_r<<endl;
#endif
  //QP matrices

  // compute Bc in continuous space for jump
  for (int h=0; h<setup->horizon; h++)
    Bc[h].setZero();

  Matrix<fpt,3,3> I_inv = I_world.inverse();

  for (int h=0; h<setup->horizon; h++){
    // get r_feet_com
    double r[12] ={update->ref_pf[4*h+0], update->ref_pf[4*h+0],update->ref_pf[4*h+2],update->ref_pf[4*h+2], -0.083, 0.083, -0.083, 0.083, update->ref_pf[4*h+1], update->ref_pf[4*h+1],update->ref_pf[4*h+3],update->ref_pf[4*h+3] }; //order: FRx, FLx, RRx, RLx, FRy, FLy, RRy, RLy, FRz, FLz, RRz, RLz
    
    for(u8 rs = 0; rs < 3; rs++)
        for(u8 c = 0; c < 4; c++)
            r_feet_com(rs,c) = r[rs*4 + c];

    for(s16 b = 0; b < 4; b++)
    {
      Bc[h].block(6,b*3,3,3) = cross_mat(I_inv,r_feet_com.col(b));
      Bc[h].block(9,b*3,3,3) = Matrix<fpt,3,3>::Identity() / rs.m;
    }
  }

  // compute A_qp, B_qp, B_qp_res
  // compute H1_res, H2_res
  // compute Phi_res

  int N1 = (800 - update-> itr_counter) / (1000 * setup->dt1);
  cout << "N1:"<< N1 << endl;
  int N2 = setup->horizon - N1;

  Timer tff;
  tff.start();
  if (N1 < setup->horizon && N1>0){
      cout <<" in hybrid prediction mode" << endl;
      c2qp_learn_hybrid(A_ct,Bc,Bc_res,setup->dt1, setup-> dt2,setup->horizon, N1, update->outf1_NN, update->outf2_NN);
  
  }
  else{
      cout <<" in single prediction mode" << endl;
      c2qp_learn(A_ct,Bc,Bc_res,setup->dt1,setup->horizon, update->outf1_NN);

    
  }

  printf("compute QP matrix time %f ms\n", tff.getMs());

  // c2qp(A_ct,B_ct_r,setup->dt,setup->horizon);

  //weights
  Matrix<fpt,13,1> full_weight;
  for(u8 i = 0; i < 12; i++)
    full_weight(i) = update->weights[i];
  full_weight(12) = 0.f;
  W.diagonal() = full_weight.replicate(setup->horizon,1);

  //trajectory
  for(s16 i = 0; i < setup->horizon; i++)
  {
    for(s16 j = 0; j < 12; j++){
      X_d(13*i+j,0) = update->traj[12*i+j];
      F_d(12*i+j,0) = update->fref[12*i+j];
    }
  }
  // cout<<"XD:\n"<<X_d<<endl;
  // cout<<"FD:\n"<<F_d<<endl;


  //note - I'm not doing the shifting here.
  int k = 0;
  for(int i = 0; i < setup->horizon; i++)
  {
    for(int j = 0; j < 4; j++)
    {
      U_b(5*k + 0) = BIG_NUMBER;
      U_b(5*k + 1) = BIG_NUMBER;
      U_b(5*k + 2) = BIG_NUMBER;
      U_b(5*k + 3) = BIG_NUMBER;
      U_b(5*k + 4) = update->ref_ct[i*4 + j] * setup->f_max;
      // U_b(5*k + 4) = setup->f_max; // Chuong: for standing only
      // U_b(5*k + 4) = update->gait[i*4 + j] * setup->f_max;
      k++; // k is equal to j when i=0
    }
  }
  
  fpt mu = 1.f/setup->mu; // original
  Matrix<fpt,5,3> f_block;

  f_block <<  mu, 0,  1.f,
    -mu, 0,  1.f,
    0,  mu, 1.f,
    0, -mu, 1.f,
    0,   0, 1.f;

  // Test update the reference contact schedule -- Chuong
  // for(int i = 0; i < setup->horizon; i++)
  // {
  //   for(int j = 0; j < 4; j++)
  //   {
  //     std::cout<< "test contact" << update->ref_ct[i*4 + j]<< std::endl;
  //   }
  //   std::cout<<"--------------"<< std::endl;
  // }
  
  
  for(s16 i = 0; i < setup->horizon*4; i++)
  {
    fmat.block(i*5,i*3,5,3) = f_block;
  }

  qH = 2*(B_qp.transpose()*W*B_qp + update->alpha*eye_12h);
  // qg = 2*B_qp.transpose()*W*(A_qp*x_0 - X_d); // orginal
  // qg = 2*B_qp.transpose()*W*(A_qp*x_0 - X_d)-2*update->alpha*eye_12h*(-F_d); // chuong add to consider F_d in cost function,
  qg = 2*B_qp.transpose()*W*(A_qp*x_0  + Hqp_res- X_d)-2*update->alpha*eye_12h*(-F_d); // learned model

  QpProblem<double> jcqp(setup->horizon*12, setup->horizon*20);
  if(update->use_jcqp) {
    jcqp.A = fmat.cast<double>();
    jcqp.P = qH.cast<double>();
    jcqp.q = qg.cast<double>();
    jcqp.u = U_b.cast<double>();
    for(s16 i = 0; i < 20*setup->horizon; i++)
      jcqp.l[i] = 0.f;

    jcqp.settings.sigma = update->sigma;
    jcqp.settings.alpha = update->solver_alpha;
    jcqp.settings.terminate = update->terminate;
    jcqp.settings.rho = update->rho;
    jcqp.settings.maxIterations = update->max_iterations;
    jcqp.runFromDense(update->max_iterations, false, false);
  } else {



    matrix_to_real(H_qpoases,qH,setup->horizon*12, setup->horizon*12);
    matrix_to_real(g_qpoases,qg,setup->horizon*12, 1);
    matrix_to_real(A_qpoases,fmat,setup->horizon*20, setup->horizon*12);
    matrix_to_real(ub_qpoases,U_b,setup->horizon*20, 1);

    for(s16 i = 0; i < 20*setup->horizon; i++)
      lb_qpoases[i] = 0.0f;

    s16 num_constraints = 20*setup->horizon;
    s16 num_variables = 12*setup->horizon;


    qpOASES::int_t nWSR = 100;


    int new_vars = num_variables;
    int new_cons = num_constraints;

    for(int i =0; i < num_constraints; i++)
      con_elim[i] = 0;

    for(int i = 0; i < num_variables; i++)
      var_elim[i] = 0;


    for(int i = 0; i < num_constraints; i++)
    {
      if(! (near_zero(lb_qpoases[i]) && near_zero(ub_qpoases[i]))) continue;
      double* c_row = &A_qpoases[i*num_variables];
      for(int j = 0; j < num_variables; j++)
      {
        if(near_one(c_row[j]))
        {
          new_vars -= 3;
          new_cons -= 5;
          int cs = (j*5)/3 -3;
          var_elim[j-2] = 1;
          var_elim[j-1] = 1;
          var_elim[j  ] = 1;
          con_elim[cs] = 1;
          con_elim[cs+1] = 1;
          con_elim[cs+2] = 1;
          con_elim[cs+3] = 1;
          con_elim[cs+4] = 1;
        }
      }
    }
    //if(new_vars != num_variables)
    if(1==1)
    {
      int var_ind[new_vars];
      int con_ind[new_cons];
      int vc = 0;
      for(int i = 0; i < num_variables; i++)
      {
        if(!var_elim[i])
        {
          if(!(vc<new_vars))
          {
            printf("BAD ERROR 1\n");
          }
          var_ind[vc] = i;
          vc++;
        }
      }
      vc = 0;
      for(int i = 0; i < num_constraints; i++)
      {
        if(!con_elim[i])
        {
          if(!(vc<new_cons))
          {
            printf("BAD ERROR 1\n");
          }
          con_ind[vc] = i;
          vc++;
        }
      }
      for(int i = 0; i < new_vars; i++)
      {
        int olda = var_ind[i];
        g_red[i] = g_qpoases[olda];
        for(int j = 0; j < new_vars; j++)
        {
          int oldb = var_ind[j];
          H_red[i*new_vars + j] = H_qpoases[olda*num_variables + oldb];
        }
      }

      for (int con = 0; con < new_cons; con++)
      {
        for(int st = 0; st < new_vars; st++)
        {
          float cval = A_qpoases[(num_variables*con_ind[con]) + var_ind[st] ];
          A_red[con*new_vars + st] = cval;
        }
      }
      for(int i = 0; i < new_cons; i++)
      {
        int old = con_ind[i];
        ub_red[i] = ub_qpoases[old];
        lb_red[i] = lb_qpoases[old];
      }

      Timer solve_timer;
      qpOASES::QProblem problem_red (new_vars, new_cons);
      qpOASES::Options op;
      op.setToMPC();
      op.printLevel = qpOASES::PL_NONE;
      problem_red.setOptions(op);
      //int_t nWSR = 50000;


      int rval = problem_red.init(H_red, g_red, A_red, NULL, NULL, lb_red, ub_red, nWSR);
      (void)rval;
      int rval2 = problem_red.getPrimalSolution(q_red);
      if(rval2 != qpOASES::SUCCESSFUL_RETURN)
        printf("failed to solve!\n");

      // printf("solve time: %.3f ms, size %d, %d\n", solve_timer.getMs(), new_vars, new_cons);


      vc = 0;
      for(int i = 0; i < num_variables; i++)
      {
        if(var_elim[i])
        {
          q_soln[i] = 0.0f;
        }
        else
        {
          q_soln[i] = q_red[vc];
          vc++;
        }
      }
    }
  }




  if(update->use_jcqp) {
    for(int i = 0; i < 12 * setup->horizon; i++) {
      q_soln[i] = jcqp.getSolution()[i];
    }
  }



#ifdef K_PRINT_EVERYTHING
  //cout<<"fmat:\n"<<fmat<<endl;
#endif



}
