#include "convexMPC_interface.h"
#include "common_types.h"
#include "SolverMPC.h"
#include <eigen3/Eigen/Dense>
#include <pthread.h>
#include <stdio.h>
#include <string.h>

#define K_NUM_LEGS 4

#define K_NUM_LEGS 4

problem_setup problem_configuration;
u8 gait_data[K_MAX_GAIT_SEGMENTS];
pthread_mutex_t problem_cfg_mt;
pthread_mutex_t update_mt;
update_data_t update;
pthread_t solve_thread;

u8 first_run = 1;

void initialize_mpc()
{
  //printf("Initializing MPC!\n");
  if(pthread_mutex_init(&problem_cfg_mt,NULL)!=0)
    printf("[MPC ERROR] Failed to initialize problem configuration mutex.\n");

  if(pthread_mutex_init(&update_mt,NULL)!=0)
    printf("[MPC ERROR] Failed to initialize update data mutex.\n");

#ifdef K_DEBUG
  printf("[MPC] Debugging enabled.\n");
    printf("[MPC] Size of problem setup struct: %ld bytes.\n", sizeof(problem_setup));
    printf("      Size of problem update struct: %ld bytes.\n",sizeof(update_data_t));
    printf("      Size of MATLAB floating point type: %ld bytes.\n",sizeof(mfp));
    printf("      Size of flt: %ld bytes.\n",sizeof(flt));
#else
  //printf("[MPC] Debugging disabled.\n");
#endif
}

void setup_problem(double dt1, double dt2, int horizon, double mu, double f_max)
{
  //mu = 0.6;
  if(first_run)
  {
    first_run = false;
    initialize_mpc();
  }

#ifdef K_DEBUG
  printf("[MPC] Got new problem configuration!\n");
    printf("[MPC] Prediction horizon length: %d\n      Force limit: %.3f, friction %.3f\n      dt: %.3f\n",
            horizon,f_max,mu,dt);
#endif

  //pthread_mutex_lock(&problem_cfg_mt);

  problem_configuration.horizon = horizon;
  problem_configuration.f_max = f_max;
  problem_configuration.mu = mu;
  // problem_configuration.dt = dt;
  problem_configuration.dt1 = dt1; // contact phase
  problem_configuration.dt2 = dt2; // flight phase

  //pthread_mutex_unlock(&problem_cfg_mt);
  resize_qp_mats(horizon);
}

//inline to motivate gcc to unroll the loop in here.
inline void mfp_to_flt(flt* dst, mfp* src, s32 n_items)
{
  for(s32 i = 0; i < n_items; i++)
    *dst++ = *src++;
}

inline void mint_to_u8(u8* dst, mint* src, s32 n_items)
{
  for(s32 i = 0; i < n_items; i++)
    *dst++ = *src++;
}

int has_solved = 0;

//void *call_solve(void* ptr)
//{
//  solve_mpc(&update, &problem_configuration);
//}
//safely copies problem data and starts the solver

void update_problem_data(int itr_counter, double* p, double* v, double* q, double* w, double* r, double pitch, double* weights, double* state_trajectory, double* force_ref, double* pf_ref, double alpha, double* ref_contact, double* NN_g, double* NN_f1, double* NN_f2)
// void update_problem_data(double* p, double* v, double* q, double* w, double* r, double pitch, double* weights, double* state_trajectory, double* force_ref, double* pf_ref, double alpha, double* ref_contact)
{
  mfp_to_flt(update.p,p,3);
  mfp_to_flt(update.v,v,3);
  mfp_to_flt(update.q,q,4);
  mfp_to_flt(update.w,w,3);
  mfp_to_flt(update.r,r,12);
  update.pitch = pitch;
  update.itr_counter = itr_counter;
  mfp_to_flt(update.weights,weights,12);
  //this is safe, the solver isn't running, and update_problem_data and setup_problem
  //are called from the same thread
  mfp_to_flt(update.traj,state_trajectory,12*problem_configuration.horizon);
  mfp_to_flt(update.fref,force_ref,12*problem_configuration.horizon);
  mfp_to_flt(update.ref_ct,ref_contact,4*problem_configuration.horizon);
  mfp_to_flt(update.ref_pf, pf_ref, 4*problem_configuration.horizon);
  mfp_to_flt(update.outg_NN, NN_g, 12*problem_configuration.horizon); // Note: the size of NN is 3x4 = 12
  mfp_to_flt(update.outf1_NN, NN_f1, 3*problem_configuration.horizon); // Note: the size of NN is 3x1 = 3
  mfp_to_flt(update.outf2_NN, NN_f2, 3*problem_configuration.horizon); // Note: the size of NN is 3x1 = 3
     
  update.alpha = alpha;

  solve_mpc(&update, &problem_configuration);
  has_solved = 1;
}


// void update_problem_data(double* p, double* v, double* q, double* w, double* r, double pitch, double* weights, double* state_trajectory, double* force_ref, double alpha, double* ref_contact)
// {
//   mfp_to_flt(update.p,p,3);
//   mfp_to_flt(update.v,v,3);
//   mfp_to_flt(update.q,q,4);
//   mfp_to_flt(update.w,w,3);
//   mfp_to_flt(update.r,r,12);
//   update.pitch = pitch;
//   mfp_to_flt(update.weights,weights,12);
//   //this is safe, the solver isn't running, and update_problem_data and setup_problem
//   //are called from the same thread
//   mfp_to_flt(update.traj,state_trajectory,12*problem_configuration.horizon);
//   mfp_to_flt(update.fref,force_ref,12*problem_configuration.horizon);
//   mfp_to_flt(update.ref_ct,ref_contact,4*problem_configuration.horizon);
//   update.alpha = alpha;
//   // mint_to_u8(update.gait,gait,4*problem_configuration.horizon);

//   solve_mpc(&update, &problem_configuration);
//   has_solved = 1;
// }

// void update_problem_data(double* p, double* v, double* q, double* w, double* r, double yaw, double* weights, double* state_trajectory, double alpha, int* gait)
// {
//   mfp_to_flt(update.p,p,3);
//   mfp_to_flt(update.v,v,3);
//   mfp_to_flt(update.q,q,4);
//   mfp_to_flt(update.w,w,3);
//   mfp_to_flt(update.r,r,12);
//   update.yaw = yaw;
//   mfp_to_flt(update.weights,weights,12);
//   //this is safe, the solver isn't running, and update_problem_data and setup_problem
//   //are called from the same thread
//   mfp_to_flt(update.traj,state_trajectory,12*problem_configuration.horizon);
//   update.alpha = alpha;
//   mint_to_u8(update.gait,gait,4*problem_configuration.horizon);

//   solve_mpc(&update, &problem_configuration);
//   has_solved = 1;
// }

void update_solver_settings(int max_iter, double rho, double sigma, double solver_alpha, double terminate, double use_jcqp) {
  update.max_iterations = max_iter;
  update.rho = rho;
  update.sigma = sigma;
  update.solver_alpha = solver_alpha;
  update.terminate = terminate;
  update.use_jcqp = (use_jcqp > 0.5);
}

void update_problem_data_floats(float* p, float* v, float* q, float* w,
                                float* r, float pitch, float* weights,
                                float* state_trajectory, float alpha, int* gait, float* ref_contact)
{
  update.alpha = alpha;
  update.pitch = pitch;
  mint_to_u8(update.gait,gait,4*problem_configuration.horizon);
  memcpy((void*)update.p,(void*)p,sizeof(float)*3);
  memcpy((void*)update.v,(void*)v,sizeof(float)*3);
  memcpy((void*)update.q,(void*)q,sizeof(float)*4);
  memcpy((void*)update.w,(void*)w,sizeof(float)*3);
  memcpy((void*)update.r,(void*)r,sizeof(float)*12);
  memcpy((void*)update.weights,(void*)weights,sizeof(float)*12);
  memcpy((void*)update.traj,(void*)state_trajectory, sizeof(float) * 12 * problem_configuration.horizon);
  memcpy((void*)update.ref_ct,(void*)ref_contact, sizeof(float) * 4 * problem_configuration.horizon);
  solve_mpc(&update, &problem_configuration);
  has_solved = 1;

}

// void update_problem_data_floats(float* p, float* v, float* q, float* w,
//                                 float* r, float yaw, float* weights,
//                                 float* state_trajectory, float alpha, int* gait)
// {
//   update.alpha = alpha;
//   update.yaw = yaw;
//   mint_to_u8(update.gait,gait,4*problem_configuration.horizon);
//   memcpy((void*)update.p,(void*)p,sizeof(float)*3);
//   memcpy((void*)update.v,(void*)v,sizeof(float)*3);
//   memcpy((void*)update.q,(void*)q,sizeof(float)*4);
//   memcpy((void*)update.w,(void*)w,sizeof(float)*3);
//   memcpy((void*)update.r,(void*)r,sizeof(float)*12);
//   memcpy((void*)update.weights,(void*)weights,sizeof(float)*12);
//   memcpy((void*)update.traj,(void*)state_trajectory, sizeof(float) * 12 * problem_configuration.horizon);
//   solve_mpc(&update, &problem_configuration);
//   has_solved = 1;

// }

double get_solution(int index)
{
  if(!has_solved) return 0.f;
  mfp* qs = get_q_soln();
  return qs[index];
}
