#ifndef _convexmpc_interface
#define _convexmpc_interface
#define K_MAX_GAIT_SEGMENTS 36

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

struct problem_setup
{
  float dt;
  float dt1, dt2;
  float mu;
  float f_max;
  int horizon;
};

struct update_data_t
{
  int itr_counter;
  float p[3];
  float v[3];
  float q[4];
  float w[3];
  float r[12];
  float pitch;
  float weights[12];
  float traj[12*K_MAX_GAIT_SEGMENTS];
  float fref[12*K_MAX_GAIT_SEGMENTS];
  float outg_NN[12*10];
  float outf1_NN[3*10];
  float outf2_NN[3*10];
  float alpha;
  float ref_ct[4*10];
  float ref_pf[4*10]; // relative foot position w.r.t body CoM in world frame
  unsigned char gait[K_MAX_GAIT_SEGMENTS];
  unsigned char hack_pad[1000];
  int max_iterations;
  double rho, sigma, solver_alpha, terminate;
  int use_jcqp;
};

EXTERNC void setup_problem(double dt1, double dt2, int horizon, double mu, double f_max);
EXTERNC void update_problem_data(int itr_counter, double* p, double* v, double* q, double* w, double* r, double pitch, double* weights, double* state_trajectory, double* force_ref, double* pf_ref, double alpha, double* ref_contact, double* NN_g, double* NN_f1, double* NN_f2); // chuong add
// EXTERNC void update_problem_data(double* p, double* v, double* q, double* w, double* r, double pitch, double* weights, double* state_trajectory, double* force_ref, double alpha, double* ref_contact); // chuong add - June 14
// EXTERNC void update_problem_data(double* p, double* v, double* q, double* w, double* r, double pitch, double* weights, double* state_trajectory, double alpha, int* gait, double* ref_contact); // chuong add
// EXTERNC void update_problem_data(double* p, double* v, double* q, double* w, double* r, double yaw, double* weights, double* state_trajectory, double alpha, int* gait); // original
EXTERNC double get_solution(int index);
EXTERNC void update_solver_settings(int max_iter, double rho, double sigma, double solver_alpha, double terminate, double use_jcqp);
EXTERNC void update_problem_data_floats(float* p, float* v, float* q, float* w,
                                        float* r, float pitch, float* weights,
                                        float* state_trajectory, float alpha, int* gait, float* ref_contact);              // chuong add
// EXTERNC void update_problem_data_floats(float* p, float* v, float* q, float* w,
//                                         float* r, float yaw, float* weights,
//                                         float* state_trajectory, float alpha, int* gait);
#endif