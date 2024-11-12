#include "FSM.h"
#include "../low_level_controller/include/Utilities/Timer.h"
//#include "../low_level_controller/include/body.h"

using namespace a1_robot;

FSM_State::FSM_State(ControlFSMData* _controlFSMData):
control(LeggedType::A1, LOWLEVEL),
_data(_controlFSMData),
Cmpc(0.001, 25, 100) 
{
  control.InitCmdData(lowCmd);
  kpMat = Mat3<double>::Zero();
  kdMat = Mat3<double>::Zero();
  init_walk_pos.setZero();
  footFeedForwardForces = Mat34<double>::Zero();
  footstepLocations = Mat34<double>::Zero();
  std::cout << "init FSM" << std::endl;
  pos.open("pos.txt");
  data_f.open("data_f.txt");
  data_pd.open("data_pd.txt");
  data_fcmd.open("data_fcmd.txt");
  com_act.open("com_act.txt");
  com_des.open("com_des.txt");
  joint_act.open("joint_act.txt");
  joint_des.open("joint_des.txt");
  foot_act.open("foot_act.txt");
  foot_des.open("foot_des.txt");
  torque.open("torque.txt");
  Voltage.open("voltage.txt");
  Current.open("current.txt");
  optitrack.open("optitrack.txt");
  pfeet_optitrack.open("pfeet_optitrack.txt");
  
  

    ////////////////////////////////////////////////////////////////////////////////////////
    // -------------------------- Reading optimization data ------------------------------//
  if (jump_MPC){
        // Read the joint velocity and position from optimization data
    std::ifstream myFile;
    myFile.open("/home/chuongusc/Documents/GitHub/Learning_MPC_Jumping/hw_execution/a1_robot_code_jumpMPC/Controllers/optimization_data/jump2D/MDC/experiments-2024/data_Q.csv"); //lab
    // myFile.open("/media/drcl/Data/a1_robot_experiments/a1_robot_code_jumpMPC/Controllers/optimization_data/jump2D/data_Q.csv");

    // If cannot open the file, report an error
    if(!myFile.is_open())
    {
        std::cout << "Could not open file for position and velocity" << std::endl;
        //return 0;
    }
    cout << "Reading Optimization Data for Position and Velocity" << endl;
    string line;
    int index = 0;
    while(getline(myFile, line))
    {
                // tau: from 0 to 11
                // FL(tau 0,1,2); FR(tau 3,4,5); RL(tau 6,7,8); RR(tau 9,10,11)
                stringstream ss(line);
                double val;
                vector<double> Q;
                while(ss >> val)
                {
                    Q.push_back(val);
                    if(ss.peek() == ',') ss.ignore();
                }
                QDes.push_back(Q);
    }
    myFile.close();



    // Read the joint torque from optimization data
    std::ifstream mytauFile("/home/chuongusc/Documents/GitHub/Learning_MPC_Jumping/hw_execution/a1_robot_code_jumpMPC/Controllers/optimization_data/jump2D/MDC/experiments-2024/data_tau.csv"); // lab
    // std::ifstream mytauFile("/media/drcl/Data/a1_robot_experiments/a1_robot_code_jumpMPC/Controllers/optimization_data/jump2D/data_tau.csv");
    // If cannot open the file, report an error
    if(!mytauFile.is_open())
    {
        std::cout << "Could not open file for torque" << endl;
        //return 0;
    }
    cout << "Reading Optimization Data for Torque" << endl;
    
    index = 0;
    while(getline(mytauFile, line))
    {
        stringstream ss(line);
        double val;
        vector<double> tau_joint;
        while(ss >> val)
        {
            tau_joint.push_back(val/2);
            if(ss.peek() == ',') ss.ignore();
        }
        tauDes.push_back(tau_joint);
    }

    mytauFile.close();



    // Read the joint velocity and position from optimization data
    std::ifstream myFFile("/home/chuongusc/Documents/GitHub/Learning_MPC_Jumping/hw_execution/a1_robot_code_jumpMPC/Controllers/optimization_data/jump2D/MDC/experiments-2024/data_F.csv");

    // If cannot open the file, report an error
    if(!myFFile.is_open())
    {
        std::cout << "Could not open file for force" << endl;
        //return 0;
    }
    cout << "Reading Optimization Data for force" << endl;
    
    index = 0;
    while(getline(myFFile, line))
    {
        stringstream ss(line);
        double val;
        vector<double> F_ref;
        while(ss >> val)
        {
            F_ref.push_back(val);
            if(ss.peek() == ',') ss.ignore();
        }
        FDes.push_back(F_ref);
    }

    myFFile.close();


    // Read the foot position from optimization data
    std::ifstream my_pfFile("/home/chuongusc/Documents/GitHub/Learning_MPC_Jumping/hw_execution/a1_robot_code_jumpMPC/Controllers/optimization_data/jump2D/MDC/experiments-2024/data_pf.csv");

    // If cannot open the file, report an error
    if(!my_pfFile.is_open())
    {
        std::cout << "Could not open file for foot position" << endl;
        //return 0;
    }
    cout << "Reading Optimization Data for foot position" << endl;
    
    index = 0;
    while(getline(my_pfFile, line))
    {
        stringstream ss(line);
        double val;
        vector<double> pf_ref;
        while(ss >> val)
        {
            pf_ref.push_back(val);
            if(ss.peek() == ',') ss.ignore();
        }
        pfDes.push_back(pf_ref);
    }

    my_pfFile.close();
  }
}


void FSM_State::Jump_MPC(){

    // reset forces and steps to 0
    Recv();
    //std::cout << "start stand" << std::endl;
    footFeedForwardForces = Mat34<double>::Zero();
    footstepLocations = Mat34<double>::Zero();
    double minForce = 5;
    double maxForce = 250;
    double contactStateScheduled[4] = {1, 1, 1, 1};


    
    double minForces[4] = {minForce, minForce, minForce, minForce};
    double maxForces[4] = {maxForce, maxForce, maxForce, maxForce};
    Vec4<double> contactphase(0.5,0.5,0.5,0.5); // default contact phase
    //Timer time;
    //time.start();
    if(!initiated){
       _data->_stateEstimator->setContactPhase(contactphase);
    }
    //double COM_weights_stance[3] = {50, 50, 50};
    //double Base_weights_stance[3] = {500, 150, 30};// for QP locomotion
    double COM_weights_stance[3] = {5, 5, 5};
    double Base_weights_stance[3] = {40, 20, 10};
    double pFeet[12], p_act[3], v_act[3], O_err[3], rpy[3],v_des[3], p_des[3],
      omegaDes[3];
    double pFeet_Optrack[12];
    double se_xfb[13], opk_quat[4];
    double kpCOM[3], kdCOM[3], kpBase[3], kdBase[3];
    double b_control[6];

    // Get the foot locations relative to COM
    Vec3<double> pFeetVecCOM, pFeetVecCOM_Optrack;
    double fMPC[4];
    double f_PD[4];
    double fcmd_mdc[4];
    double fcmd[4];
    Eigen::Matrix3d R_b_world; // obtain from quat_b_world

    // convert torque PD to force PD:
    RotMat<double> Rt;
    Mat3<double> Jt, Jt_Rt;
    Vec3<double> force_PD, force_cmd, force_mdc;
    double fPD[12], f_cmd[12], fcmdmdc[12];

    
    for(int i = 0; i < 3; i++){
      p_des[i] = 0;
      v_des[i] = 0;
      omegaDes[i] = 0;
      rpy[i] = 0;
    
    }

    // Motor Dynamics
    //V= IR+L*dI/dt+ke*w = IR+ke*w = T*R/kt +ke*w = T*R/k+k*w; here Kt=Ke=K
    double Kt = 0.118;
    double R=25 * Kt * Kt; 
    double K=0.11; 
    double gr=8.5;
    double voltage[4]; // Motor voltage in Front thigh, front calf, rear thigh, rear calf.
    double current[4]; // Motor current in Front thigh, front calf, rear thigh, rear calf.
    double torque_max=33.5;
    double _voltage_max = 21.5;
    double _gear_ratio = 8.5;
    double _joint_vel_limit = 21;
    double _joint_torque_max = 33.5;
    double _R_motor = 25 * Kt * Kt;

    double Kp = 400;
    double Kd = 4; 
    double Hip_Kp = 400;
    double Hip_Kd =4;
    double Kp_l=5;
    double Kd_l=1;
    double init_Pos[12] = {0, 1.2566, -2.355, 0, 1.2566, -2.355, 0, 1.2566, -2.355, 0, 1.2566, -2.355}; // d60cm
    // double init_Pos[12] = {0, 1.1310, -2.1206, 0, 1.1310, -2.1206, 0, 1.1310, -2.1206, 0, 1.1310, -2.1206}; // d60 cm old+backflip
    double threshold=8;
    double currentPos[12], percent;

    int N_TrajRef=1200;
    int standing=3000;
    int startjump=4000;
    int pose_time=100;
    int dc=500;
    int sc=300;
    int fl=400;

    double box_height=0;

    int idx_pose=N_TrajRef-pose_time;
    int idx=0;

    double FR=0; double FL=0; double RR=0; double RL=0; // contact state
    bool jump_with_deltaMPC = false; // 
    bool jump_with_MPC = true; // 
    bool jump_with_MPC_taudes = false;

    //double v_des[3] = {0, 0, 0};
    p_des[2] = 0.15; // standup height for A1
    
    //std::cout << ros::Time::now() << std::endl;
    // rate.reset();
    //std::cout << "get leg state" << std::endl;
    _data->_desiredStateCommand->convertToStateCommands();
    _data->_legController->updateData();
    //std::cout << "get state" << std::endl;
    _data->_stateEstimator->run();



    // optitrack << OptiTrack_data[0] << "  " << OptiTrack_data[1] << " " << OptiTrack_data[2] << " " << OptiTrack_data[3] << " " << OptiTrack_data[4] << " " << OptiTrack_data[5] << " " << OptiTrack_data[6] << std::endl;;

    if(motiontime > 1000){
         initiated = true;
    }

    if (motiontime <startjump){  // Initialization
          runQP=false;
          // std::cout << "FLfootCallback: " << lowState.footForce[1] << std::endl;
          // std::cout << "RLfootCallback: " << lowState.footForce[3] << std::endl;
          //double contactStateScheduled[4]={1,1,1,1}; // all legs are stance legs
        for(int i = 0; i < 4; i++){
            _data->_legController->commands[i].kpJoint = Vec3<double>(300,300,300).asDiagonal();
            _data->_legController->commands[i].kdJoint = Vec3<double>(3,3,3).asDiagonal();
            _data->_legController->commands[i].kpCartesian << Mat3<double>::Zero();
            _data->_legController->commands[i].kdCartesian << Mat3<double>::Zero();
        }


        percent = (double) motiontime/ startjump; 
          //  std ::cout << "percent: " << percent << std::endl; 
           for (int i=0; i<4; i++){ 
               for(int j=0; j<3; j++){ 
                 currentPos[3*i+j] = _data->_legController->data[i].q(j); 
                 _data->_legController->commands[i].qDes(j) = currentPos[3*i+j]*(1-percent) + init_Pos[3*i+j]*percent; 
               } 
           } 
    }

    /*======================== Locomotion ====================*/

  if(motiontime>=startjump){
     optitrack << OptiTrack_data[0] << "  " << OptiTrack_data[1] << " " << OptiTrack_data[2] << " " << OptiTrack_data[3] << " " << OptiTrack_data[4] << " " << OptiTrack_data[5] << " " << OptiTrack_data[6] << std::endl;;
          // get rotation matrix from Optitrack

        // It is important to note the order of quaternion
        for (int i = 0; i < 4; i++) {
          opk_quat[i] = OptiTrack_data[i+3];
          // cout << "opk_quat:" << opk_quat << std::endl;
        }

        // opk_quat[0] = OptiTrack_data[6];
        // opk_quat[1] = OptiTrack_data[3];
        // opk_quat[2] = OptiTrack_data[4];
        // opk_quat[3] = OptiTrack_data[5];

        // cout <<"optitrack_data:" <<  OptiTrack_data[0] << "  " << OptiTrack_data[1] << " " << OptiTrack_data[2] << " " << OptiTrack_data[3] << " " << OptiTrack_data[4] << " " << OptiTrack_data[5] << " " << OptiTrack_data[6] << std::endl;;
        // from wikipedia
        R_b_world(0, 0) = 1 - 2 * opk_quat[2] * opk_quat[2] - 2 * opk_quat[3] * opk_quat[3];
        R_b_world(0, 1) = 2 * opk_quat[1] * opk_quat[2] - 2 * opk_quat[0] * opk_quat[3];
        R_b_world(0, 2) = 2 * opk_quat[1] * opk_quat[3] + 2 * opk_quat[0] * opk_quat[2];
        R_b_world(1, 0) = 2 * opk_quat[1] * opk_quat[2] + 2 * opk_quat[0] * opk_quat[3];
        R_b_world(1, 1) = 1 - 2 * opk_quat[1] * opk_quat[1] - 2 * opk_quat[3] * opk_quat[3];
        R_b_world(1, 2) = 2 * opk_quat[2] * opk_quat[3] - 2 * opk_quat[1] * opk_quat[0];
        R_b_world(2, 0) = 2 * opk_quat[1] * opk_quat[3] - 2 * opk_quat[2] * opk_quat[0];
        R_b_world(2, 1) = 2 * opk_quat[2] * opk_quat[3] + 2 * opk_quat[1] * opk_quat[0];
        R_b_world(2, 2) = 1 - 2 * opk_quat[1] * opk_quat[1] - 2 * opk_quat[2] * opk_quat[2];
  }
   if(motiontime>=startjump && motiontime<startjump+N_TrajRef-pose_time){


      // double x_act = _data->_stateEstimator->getResult().position(0);
      // double y_act = _data->_stateEstimator->getResult().position(1);
   
     Cmpc.run_Jump(*_data, QDes, FDes, pfDes); // MPC Controller, works

    idx= motiontime-startjump;
    std::cout<< "idx: " << idx <<std::endl;
    _data->_legController->commands[0].tau << 0, -tauDes[0][idx], -tauDes[1][idx];// front right leg
    _data->_legController->commands[1].tau << _data->_legController->commands[0].tau;// front right leg
    _data->_legController->commands[2].tau << 0, -tauDes[2][idx], -tauDes[3][idx];// rear right leg
    _data->_legController->commands[3].tau << _data->_legController->commands[2].tau;// rear right leg

    _data->_legController->commands[0].qDes << 0, -QDes[3][idx], -QDes[4][idx]; // front right leg
    _data->_legController->commands[1].qDes << _data->_legController->commands[0].qDes;
    _data->_legController->commands[2].qDes << 0, -QDes[5][idx], -QDes[6][idx];// rear right leg
    _data->_legController->commands[3].qDes << _data->_legController->commands[2].qDes;

    _data->_legController->commands[0].qdDes << 0, -QDes[10][idx], -QDes[11][idx]; // front right leg
    _data->_legController->commands[1].qdDes << _data->_legController->commands[0].qdDes;
    _data->_legController->commands[2].qdDes << 0, -QDes[12][idx], -QDes[13][idx];// rear right leg
     _data->_legController->commands[3].qdDes << _data->_legController->commands[2].qdDes;

    double Fjump_des[12]={0,0,0,0,0,0,0,0,0,0,0,0};

    Fjump_des[0]=FDes[0][idx]/2;
    Fjump_des[2]=FDes[1][idx]/2;
    Fjump_des[3]=FDes[0][idx]/2;
    Fjump_des[5]=FDes[1][idx]/2;
    Fjump_des[6]=FDes[2][idx]/2;
    Fjump_des[8]=FDes[3][idx]/2;
    Fjump_des[9]=FDes[2][idx]/2;
    Fjump_des[11]=FDes[3][idx]/2;


    // // // convert force to torque

    if(jump_with_MPC){
       for (int leg=0; leg<4; leg++)
          {
            Vec3<double> legTorque(0, 0, 0);
            Vec3<double> footForce = _data->_legController->commands[leg].feedforwardForce; // force in body frame
            // std::cout << "footForce_chuong" << footForce << std::endl;
            legTorque +=_data->_legController->data[leg].J.transpose()*footForce;
            _data->_legController->commands[leg].tau = legTorque;
            // std::cout<< "command tau in FSM:" << _data->_legController->commands[i].tau << std::endl;

          }

    }

        Vec3<double> pFeetVec;
        // compute pFeet

        for (int leg = 0; leg < 4; leg++) {
          // computeLegJacobianAndPosition(&_data->_quadruped, _data->_legController->data[leg].q,
          //                           (Mat3<double>*)nullptr, &pFeetVec, leg);
          //pFeetVecCOM = _data->_stateEstimator->getResult().rBody.transpose() *
                    //(_data->_quadruped->getHipLocation(leg) + pFeetVec);

          pFeetVecCOM =  _data->_stateEstimator->getResult().rBody.transpose() *
          (_data->_quadruped->getHipLocation(leg) + _data->_legController->data[leg].p);

          pFeetVecCOM_Optrack = R_b_world *
          (_data->_quadruped->getHipLocation(leg) + _data->_legController->data[leg].p); // rBody.transpose = R_b_world

          pFeet[leg * 3] = pFeetVecCOM[0];
          pFeet[leg * 3 + 1] = pFeetVecCOM[1];
          pFeet[leg * 3 + 2] = pFeetVecCOM[2];
          pFeet_Optrack[leg * 3] = pFeetVecCOM_Optrack[0];
          pFeet_Optrack[leg * 3 + 1] = pFeetVecCOM_Optrack[1];
          pFeet_Optrack[leg * 3 + 2] = pFeetVecCOM_Optrack[2];
          // std::cout << "leg" << leg << std::endl;
          // std::cout << "pFeet" << pFeetVecCOM << std::endl;
          // std::cout << "pFeet_Optrack" << pFeetVecCOM_Optrack << std::endl;
        }



        for (int leg= 0; leg< 4; leg++){
            // get tau_PD first
            Vec3<double> tau_PD = _data->_legController->commands[leg].kpJoint *(_data->_legController->commands[leg].qDes - _data->_legController->data[leg].q) +
                                  _data->_legController->commands[leg].kdJoint *(_data->_legController->commands[leg].qdDes - _data->_legController->data[leg].qd);

            _data->_legController->commands[leg].tau_mdc = _data->_legController->commands[leg].tau;
            for (int j = 0; j < 3; j++)
            {
                voltage[leg * 3 + j] = _data->_legController->commands[leg].tau(j) * _R_motor / (Kt * _gear_ratio) + _data ->_legController->data[leg].qd(j) * _gear_ratio * Kt;
                // cout << "voltage:" << voltage[i * 3 + j] << std::endl;
                if (voltage[leg * 3 + j] > _voltage_max)
                {
                    _data->_legController->commands[leg].tau_mdc[j] = (_voltage_max - 1.0 * _data ->_legController->data[leg].qd(j) * _gear_ratio * Kt) * (Kt * _gear_ratio / _R_motor);
                }
                if (voltage[leg * 3 + j] < -1.0 * _voltage_max)
                {
                    _data->_legController->commands[leg].tau_mdc[j] = (-1.0 * _voltage_max - _data ->_legController->data[leg].qd(j) * _gear_ratio * Kt) * (Kt * _gear_ratio / _R_motor);
                }
            }
            Rt = _data->_stateEstimator->getResult().rBody;  // rBody = R.transpose()
            Jt = _data->_legController->data[leg].J.transpose();
            Jt_Rt = Jt * Rt;
            force_PD = -Jt_Rt.inverse()*tau_PD;
            force_cmd = -Jt_Rt.inverse()*_data->_legController->commands[leg].tau;
            // force_mdc = -Jt_Rt.inverse()*_data->_legController->commands[leg].tau_mdc;
            for (int j=0; j<3; j++){
              fPD[leg*3+j] = force_PD[j];
              f_cmd[leg*3+j] = force_cmd[j];
              // fcmdmdc[leg*3+j] = force_mdc[j];
            }
            // std::cout << "leg" << leg << "force PD:" << fPD[leg*3] << ", " << fPD[leg*3 + 1] << ", " << fPD[leg*3 + 2] << std::endl;

        }


        // save to fMPC to export data

        fMPC[0] = 2*(_data->_legController->commands[0].force_MPC[0]);// + fPD[0];
        fMPC[1] = 2*(_data->_legController->commands[0].force_MPC[2]);// + fPD[2];
        fMPC[2] = 2*(_data->_legController->commands[2].force_MPC[0]);// + fPD[6];
        fMPC[3] = 2*(_data->_legController->commands[2].force_MPC[2]);// + fPD[8];

        f_PD[0] = 2*(fPD[0]);
        f_PD[1] = 2*(fPD[2]);
        f_PD[2] = 2*(fPD[6]);
        f_PD[3] = 2*(fPD[8]);

    runQP = false;
    // firstrunQP = true;
   
   }
  // LANDING .....
  // Control front swing legs: PD controller

  if (motiontime>=startjump+N_TrajRef-pose_time && motiontime<=startjump+N_TrajRef-pose_time+50){
          // use PD for swing legs

          _data->_legController->commands[0].tau << 0, 0, 0;// front right leg
          _data->_legController->commands[1].tau << 0, 0, 0; // front left leg

          _data->_legController->commands[0].qDes << 0, -QDes[3][idx_pose]-0.5, -QDes[4][idx_pose]; // move front right leg
          _data->_legController->commands[1].qDes <<_data->_legController->commands[0].qDes; // front left leg

          _data->_legController->commands[0].qdDes << 0, 0, 0; // front right leg
          _data->_legController->commands[1].qdDes << 0, 0, 0; // front left leg

          for (int i=0; i<2; i++){
            _data->_legController->commands[i].kpJoint << 300, 0, 0,
                                                          0, 80, 0,
                                                          0, 0, 80;
            _data->_legController->commands[i].kdJoint << 3, 0, 0,
                                                          0, 1, 0,
                                                          0, 0, 1;

          }
          runQP=false;
  }

    if(motiontime >=startjump+N_TrajRef-pose_time+50){

          // std::cout<< "QP activated" << std::endl;

          // Get contact state
          FR=lowState.footForce[0];
          FL=lowState.footForce[1];
          RR=lowState.footForce[2];
          RL=lowState.footForce[3];
          if (FR>threshold){
            FR=1;
            _data->_legController->commands[0].kpJoint << Kp_l, 0, 0,
                                                          0, Kp_l, 0,
                                                          0, 0, Kp_l;
            _data->_legController->commands[0].kdJoint << Kd_l, 0, 0,
                                                          0, Kd_l, 0,
                                                          0, 0, Kd_l;
            _data->_legController->commands[0].tau << 0, 0, 0;// front right leg
          }
          else{
            FR=0;
          }
          if (FL>threshold){
            FL=1;
            _data->_legController->commands[1].kpJoint << Kp_l, 0, 0,
                                                          0, Kp_l, 0,
                                                          0, 0, Kp_l;
            _data->_legController->commands[1].kdJoint << Kd_l, 0, 0,
                                                          0, Kd_l, 0,
                                                          0, 0, Kd_l;
            _data->_legController->commands[1].tau << 0, 0, 0;// front right leg
          }
          else{
            FL=0;
          }
          if (RL>threshold){ // since there some problem with rear right foot sensor
            RR=1;
            _data->_legController->commands[2].kpJoint << Kp_l, 0, 0,
                                                          0, Kp_l, 0,
                                                          0, 0, Kp_l;
            _data->_legController->commands[2].kdJoint << Kd_l, 0, 0,
                                                          0, Kd_l, 0,
                                                          0, 0, Kd_l;
            _data->_legController->commands[2].tau << 0, 0, 0;// front right leg
            
          }
          else{
            RL=0;
          }
          if (RL>threshold){
            RL=1;
            _data->_legController->commands[3].kpJoint << Kp_l, 0, 0,
                                                          0, Kp_l, 0,
                                                          0, 0, Kp_l;
            _data->_legController->commands[3].kdJoint << Kd_l, 0, 0,
                                                          0, Kd_l, 0,
                                                          0, 0, Kd_l;
            _data->_legController->commands[3].tau << 0, 0, 0;// front right leg
          }
          else{
            RL=0;
          }
          
          // double contactStateScheduled[4]={FR,FL,RR,RL};
          double contactStateScheduled[4]={1,1,1,1};
          std::cout << "contactStateScheduled: " << contactStateScheduled[0] << "," << contactStateScheduled[1]<<"," << contactStateScheduled[2]<< "," << contactStateScheduled[3]<< std::endl;
          runQP=true;
    }

    if(motiontime >=startjump+N_TrajRef){ // make sure all joint has low gain in case all footsensor has issues

      for(int i=0; i<4; i++){
          _data->_legController->commands[i].kpJoint << Kp_l, 0, 0,
                                                          0, Kp_l, 0,
                                                          0, 0, Kp_l;
          _data->_legController->commands[i].kdJoint << Kd_l, 0, 0,
                                                          0, Kd_l, 0,
                                                          0, 0, Kd_l;
      }

    }
    
    if (motiontime>=startjump+N_TrajRef-pose_time){ 

        // compute pFeet

        for (int leg = 0; leg < 4; leg++) {
          // computeLegJacobianAndPosition(&_data->_quadruped, _data->_legController->data[leg].q,
          //                           (Mat3<double>*)nullptr, &pFeetVec, leg);
          //pFeetVecCOM = _data->_stateEstimator->getResult().rBody.transpose() *
                    //(_data->_quadruped->getHipLocation(leg) + pFeetVec);

          pFeetVecCOM =  _data->_stateEstimator->getResult().rBody.transpose() *
          (_data->_quadruped->getHipLocation(leg) + _data->_legController->data[leg].p);

          pFeetVecCOM_Optrack = R_b_world *
          (_data->_quadruped->getHipLocation(leg) + _data->_legController->data[leg].p); // rBody.transpose = R_b_world

          pFeet[leg * 3] = pFeetVecCOM[0];
          pFeet[leg * 3 + 1] = pFeetVecCOM[1];
          pFeet[leg * 3 + 2] = pFeetVecCOM[2];
          pFeet_Optrack[leg * 3] = pFeetVecCOM_Optrack[0];
          pFeet_Optrack[leg * 3 + 1] = pFeetVecCOM_Optrack[1];
          pFeet_Optrack[leg * 3 + 2] = pFeetVecCOM_Optrack[2];
          // std::cout << "leg" << leg << std::endl;
          // std::cout << "pFeet" << pFeetVecCOM << std::endl;
          // std::cout << "pFeet_Optrack" << pFeetVecCOM_Optrack << std::endl;
        }


        for (int leg= 0; leg< 4; leg++){
            // get tau_PD first
            Vec3<double> tau_PD = _data->_legController->commands[leg].kpJoint *(_data->_legController->commands[leg].qDes - _data->_legController->data[leg].q) +
                                  _data->_legController->commands[leg].kdJoint *(_data->_legController->commands[leg].qdDes - _data->_legController->data[leg].qd);

            for (int j = 0; j < 3; j++)
            {
                voltage[leg * 3 + j] = _data->_legController->commands[leg].tau(j) * _R_motor / (Kt * _gear_ratio) + _data ->_legController->data[leg].qd(j) * _gear_ratio * Kt;
                // cout << "voltage:" << voltage[i * 3 + j] << std::endl;
                if (voltage[leg * 3 + j] > _voltage_max)
                {
                    _data->_legController->commands[leg].tau_mdc[j] = (_voltage_max - 1.0 * _data ->_legController->data[leg].qd(j) * _gear_ratio * Kt) * (Kt * _gear_ratio / _R_motor);
                }
                if (voltage[leg * 3 + j] < -1.0 * _voltage_max)
                {
                    _data->_legController->commands[leg].tau_mdc[j] = (-1.0 * _voltage_max - _data ->_legController->data[leg].qd(j) * _gear_ratio * Kt) * (Kt * _gear_ratio / _R_motor);
                }
            }
            Rt = _data->_stateEstimator->getResult().rBody;
            Jt = _data->_legController->data[leg].J.transpose();
            Jt_Rt = Jt * Rt;
            force_PD = -Jt_Rt.inverse()*tau_PD;
            force_cmd = -Jt_Rt.inverse()*_data->_legController->commands[leg].tau;
            // force_mdc = -Jt_Rt.inverse()*_data->_legController->commands[leg].tau_mdc;
            for (int j=0; j<3; j++){
              fPD[leg*3+j] = force_PD[j];
              f_cmd[leg*3+j] = force_cmd[j];
              // fcmdmdc[leg*3+j] = force_mdc[j];
            }
            // std::cout << "leg" << leg << "force PD:" << fPD[leg*3] << ", " << fPD[leg*3 + 1] << ", " << fPD[leg*3 + 2] << std::endl;

        }

        f_PD[0] = 2*fPD[0];
        f_PD[1] = 2*fPD[2];
        f_PD[2] = 2*fPD[6];
        f_PD[3] = 2*fPD[8];

      }

    /* =================================================================*/
    // if(_data->_desiredStateCommand->data.mode == 1  && initiated){
    if(runQP){
       _data->_stateEstimator->setContactPhase(contactphase);
      if(firstrunQP){
        // rpy[2] = _data->_stateEstimator->getResult().rpy(2);
        init_yaw = _data->_stateEstimator->getResult().rpy(2);
        firstrunQP = false;
      }
      // set rpy command
      rpy[0] = 0;
      rpy[1] = 0;
      rpy[2] = init_yaw;

      for (int i = 0; i < 4; i++) {
        se_xfb[i] = _data->_stateEstimator->getResult().orientation(i);
      }

      for (int i = 0; i < 3; i++) {
        // rpy[i] = 0;
        //rpy[i] = _data->_stateEstimator->getResult().rpy(i);
        //p_des[i] = _data->_stateEstimator->getResult().position(i);
        p_act[i] = _data->_stateEstimator->getResult().position(i);
        
        v_act[i] = _data->_stateEstimator->getResult().vWorld(i);
        // v_des[i] = 0;
        //v_des[2] = 0.005;

        se_xfb[4 + i] = _data->_stateEstimator->getResult().position(i);
        se_xfb[7 + i] = _data->_stateEstimator->getResult().omegaBody(i);
        se_xfb[10 + i] = _data->_stateEstimator->getResult().vWorld(i);
        
    // Set the translational and orientation gains
      // Set the translational and orientation gains
        // kpCOM[i] = 25;   //_data->controlParameters->kpCOM(i);
        // kdCOM[i] = 5;     //_data->controlParameters->kdCOM(i);
        // kpBase[i] = 200;     //_data->controlParameters->kpBase(i);
        // kdBase[i] = 10; //  _data->controlParameters->kdBase(i);

        kpCOM[i] = 30;   //_data->controlParameters->kpCOM(i);
        kdCOM[i] = 10;     //_data->controlParameters->kdCOM(i);
        kpBase[i] = 80;     //_data->controlParameters->kpBase(i);
        kdBase[i] = 20; //  _data->controlParameters->kdBase(i);

        }
      // kpCOM[2] = 55;
      // // kdCOM[2] = 20;
      // kpBase[0] = 600;
      // kdBase[0] = 5;
      // kpBase[1] = 400;

      kpCOM[2] = 50;
      kdCOM[2] = 20;
      kpBase[0] = 300;
      //kdBase[0] = 5;
      kpBase[1] = 200;


    //Vec3<double> pFeetVec;
    Vec3<double> pFeetVecCOM;

    // Get the foot locations relative to COM
      for (int leg = 0; leg < 4; leg++) {
        // computeLegJacobianAndPosition(&_data->_quadruped, _data->_legController->data[leg].q,
        //                           (Mat3<double>*)nullptr, &pFeetVec, leg);
        //pFeetVecCOM = _data->_stateEstimator->getResult().rBody.transpose() *
                  //(_data->_quadruped->getHipLocation(leg) + pFeetVec);

        pFeetVecCOM =  _data->_stateEstimator->getResult().rBody.transpose() *
        (_data->_quadruped->getHipLocation(leg) + _data->_legController->data[leg].p);

        pFeetVecCOM_Optrack = R_b_world *
        (_data->_quadruped->getHipLocation(leg) + _data->_legController->data[leg].p); // rBody.transpose = R_b_world


        pFeet[leg * 3] = pFeetVecCOM[0];
        pFeet[leg * 3 + 1] = pFeetVecCOM[1];
        pFeet[leg * 3 + 2] = pFeetVecCOM[2];
        pFeet_Optrack[leg * 3] = pFeetVecCOM_Optrack[0];
        pFeet_Optrack[leg * 3 + 1] = pFeetVecCOM_Optrack[1];
        pFeet_Optrack[leg * 3 + 2] = pFeetVecCOM_Optrack[2];
        //std::cout << "pFeet" << leg << std::endl;
      }

      p_des[0] = p_act[0] + (pFeet[0] + pFeet[3] + pFeet[6] + pFeet[9])/4.0;
      p_des[1] = p_act[1] + (pFeet[1] + pFeet[4] + pFeet[7] + pFeet[10])/4.0;
    //  myfile << "\n";
     // std::cout << j << std::endl;
     //std::cout << "run QP" << std::endl;
    balanceController.set_alpha_control(0.01);
    balanceController.set_friction(0.2);
    balanceController.set_mass(_data->_quadruped->mass);
    balanceController.set_wrench_weights(COM_weights_stance, Base_weights_stance);
    balanceController.set_PDgains(kpCOM, kdCOM, kpBase, kdBase);
    balanceController.set_desiredTrajectoryData(rpy, p_des, omegaDes, v_des);
    balanceController.SetContactData(contactStateScheduled, minForces, maxForces);
    balanceController.updateProblemData(se_xfb, pFeet, p_des, p_act, v_des, v_act,
                                      O_err, _data->_stateEstimator->getResult().rpy(2));
   // balanceController.print_QPData();
    double fOpt[12];
    balanceController.solveQP_nonThreaded(fOpt);
    //balanceController.get_b_matrix(b_control);
    //b_des << b_control[2] << "\n";

  // Publish the results over ROS
  // balanceController.publish_data_lcm();

  // Copy the results to the feed forward forces
    
     //_data->_stateEstimator->run();


    for (int leg = 0; leg < 4; leg++) {
        footFeedForwardForces.col(leg) << fOpt[leg * 3], fOpt[leg * 3 + 1],
        fOpt[leg * 3 + 2]; // force in world frame, need to convert to body frame

        _data->_legController->commands[leg].feedforwardForce = footFeedForwardForces.col(leg);
        //_data->_stateEstimator->getResult().rBody.transpose() * footFeedForwardForces.col(leg); 
       // QP << _data->_legController->commands[leg].feedforwardForce[2] << " ";
    }
    
    //_data->_legController->updateCommand();

    // }
    }

      if (motiontime >=startjump){

       int idx=motiontime-startjump;

        double final_torque_t0=_data ->_legController->commands[0].tau_mdc(1)+ Kp*(_data ->_legController->commands[0].qDes(1)-_data ->_legController->data[0].q(1))+Kd*(_data ->_legController->commands[0].qdDes(1)-_data ->_legController->data[0].qd(1));
        double final_torque_t1=_data ->_legController->commands[1].tau_mdc(1)+ Kp*(_data ->_legController->commands[1].qDes(1)-_data ->_legController->data[1].q(1))+Kd*(_data ->_legController->commands[1].qdDes(1)-_data ->_legController->data[1].qd(1));
        double final_torque_t2=_data ->_legController->commands[2].tau_mdc(1)+ Kp*(_data ->_legController->commands[2].qDes(1)-_data ->_legController->data[2].q(1))+Kd*(_data ->_legController->commands[2].qdDes(1)-_data ->_legController->data[2].qd(1));
        double final_torque_t3=_data ->_legController->commands[3].tau_mdc(1)+ Kp*(_data ->_legController->commands[3].qDes(1)-_data ->_legController->data[3].q(1))+Kd*(_data ->_legController->commands[3].qdDes(1)-_data ->_legController->data[3].qd(1));
        double final_torque_c0=_data ->_legController->commands[0].tau_mdc(2)+ Kp*(_data ->_legController->commands[0].qDes(2)-_data ->_legController->data[0].q(2))+Kd*(_data ->_legController->commands[0].qdDes(2)-_data ->_legController->data[0].qd(2));
        double final_torque_c1=_data ->_legController->commands[1].tau_mdc(2)+ Kp*(_data ->_legController->commands[1].qDes(2)-_data ->_legController->data[1].q(2))+Kd*(_data ->_legController->commands[1].qdDes(2)-_data ->_legController->data[1].qd(2));
        double final_torque_c2=_data ->_legController->commands[2].tau_mdc(2)+ Kp*(_data ->_legController->commands[2].qDes(2)-_data ->_legController->data[2].q(2))+Kd*(_data ->_legController->commands[2].qdDes(2)-_data ->_legController->data[2].qd(2));
        double final_torque_c3=_data ->_legController->commands[3].tau_mdc(2)+ Kp*(_data ->_legController->commands[3].qDes(2)-_data ->_legController->data[3].q(2))+Kd*(_data ->_legController->commands[3].qdDes(2)-_data ->_legController->data[3].qd(2));
        
        double total_torque_t0=_data ->_legController->commands[0].tau(1)+ Kp*(_data ->_legController->commands[0].qDes(1)-_data ->_legController->data[0].q(1))+Kd*(_data ->_legController->commands[0].qdDes(1)-_data ->_legController->data[0].qd(1));
        double total_torque_t1=_data ->_legController->commands[1].tau(1)+ Kp*(_data ->_legController->commands[1].qDes(1)-_data ->_legController->data[1].q(1))+Kd*(_data ->_legController->commands[1].qdDes(1)-_data ->_legController->data[1].qd(1));
        double total_torque_t2=_data ->_legController->commands[2].tau(1)+ Kp*(_data ->_legController->commands[2].qDes(1)-_data ->_legController->data[2].q(1))+Kd*(_data ->_legController->commands[2].qdDes(1)-_data ->_legController->data[2].qd(1));
        double total_torque_t3=_data ->_legController->commands[3].tau(1)+ Kp*(_data ->_legController->commands[3].qDes(1)-_data ->_legController->data[3].q(1))+Kd*(_data ->_legController->commands[3].qdDes(1)-_data ->_legController->data[3].qd(1));
        double total_torque_c0=_data ->_legController->commands[0].tau(2)+ Kp*(_data ->_legController->commands[0].qDes(2)-_data ->_legController->data[0].q(2))+Kd*(_data ->_legController->commands[0].qdDes(2)-_data ->_legController->data[0].qd(2));
        double total_torque_c1=_data ->_legController->commands[1].tau(2)+ Kp*(_data ->_legController->commands[1].qDes(2)-_data ->_legController->data[1].q(2))+Kd*(_data ->_legController->commands[1].qdDes(2)-_data ->_legController->data[1].qd(2));
        double total_torque_c2=_data ->_legController->commands[2].tau(2)+ Kp*(_data ->_legController->commands[2].qDes(2)-_data ->_legController->data[2].q(2))+Kd*(_data ->_legController->commands[2].qdDes(2)-_data ->_legController->data[2].qd(2));
        double total_torque_c3=_data ->_legController->commands[3].tau(2)+ Kp*(_data ->_legController->commands[3].qDes(2)-_data ->_legController->data[3].q(2))+Kd*(_data ->_legController->commands[3].qdDes(2)-_data ->_legController->data[3].qd(2));
        

        com_act << _data->_stateEstimator->getResult().vWorld(0) << " " << _data->_stateEstimator->getResult().vWorld(1) << " " <<  _data->_stateEstimator->getResult().vWorld(2) << " " 
        <<_data->_stateEstimator->getResult().position(0) << " " << _data->_stateEstimator->getResult().position(1) << " " << _data->_stateEstimator->getResult().position(2) << " " 
        << _data->_stateEstimator->getResult().rpy(0) << " " << _data->_stateEstimator->getResult().rpy(1) << " " << _data->_stateEstimator->getResult().rpy(2) << std::endl;

        com_des << QDes[7][idx] << " " << 0 << " " << QDes[8][idx] << " " 
        << QDes[0][idx] << " " << 0 << " " << QDes[1][idx] << " "
        << 0 << " " << QDes[2][idx] << " " << 0 << std::endl;

        // _data->_legController->commands[0].tau << 0, -tauDes[0][idx], -tauDes[1][idx];// front right leg
        // _data->_legController->commands[1].tau << _data->_legController->commands[0].tau; // front left leg
        // _data->_legController->commands[2].tau << 0, -tauDes[2][idx], -tauDes[3][idx];// rear right leg
        // _data->_legController->commands[3].tau << _data->_legController->commands[2].tau; // rear left leg

        torque << _data ->_legController->commands[0].tau_mdc(0) <<" "<< _data ->_legController->commands[1].tau_mdc(0) <<" "<<_data ->_legController->commands[2].tau_mdc(0)<<" "<<_data ->_legController->commands[3].tau_mdc(0)<<" "
    	  << _data ->_legController->commands[0].tau_mdc(1) <<" "<< _data ->_legController->commands[1].tau_mdc(1) <<" "<<_data ->_legController->commands[2].tau_mdc(1)<<" "<<_data ->_legController->commands[3].tau_mdc(1)<<" "
    	  << _data ->_legController->commands[0].tau_mdc(2) <<" "<< _data ->_legController->commands[1].tau_mdc(2) <<" "<<_data ->_legController->commands[2].tau_mdc(2)<<" "<<_data ->_legController->commands[3].tau_mdc(2)<<" "
        << final_torque_t0 <<" " << final_torque_t1 <<" " << final_torque_t2 <<" " << final_torque_t3 <<" " 
        << final_torque_c0 <<" " << final_torque_c1 <<" " << final_torque_c2 <<" " << final_torque_c3 << " "
        <<_data ->_legController->commands[0].tau(0) <<" "<< _data ->_legController->commands[1].tau(0) <<" "<<_data ->_legController->commands[2].tau(0)<<" "<<_data ->_legController->commands[3].tau(0)<<" "
    	  << _data ->_legController->commands[0].tau(1) <<" "<< _data ->_legController->commands[1].tau(1) <<" "<<_data ->_legController->commands[2].tau(1)<<" "<<_data ->_legController->commands[3].tau(1)<<" "
    	  << _data ->_legController->commands[0].tau(2) <<" "<< _data ->_legController->commands[1].tau(2) <<" "<<_data ->_legController->commands[2].tau(2)<<" "<<_data ->_legController->commands[3].tau(2)<<" "
        << total_torque_t0 <<" " << total_torque_t1 <<" " << total_torque_t2 <<" " << total_torque_t3 <<" " 
        << total_torque_c0 <<" " << total_torque_c1 <<" " << total_torque_c2 <<" " << total_torque_c3 <<std::endl;

        joint_act << _data ->_legController->data[0].q(0) <<" "<< _data ->_legController->data[1].q(0) <<" "<<_data ->_legController->data[2].q(0)<<" "<<_data ->_legController->data[3].q(0)<<" "
        <<_data ->_legController->data[0].q(1) <<" "<< _data ->_legController->data[1].q(1) <<" "<<_data ->_legController->data[2].q(1)<<" "<<_data ->_legController->data[3].q(1)<<" " 
        <<_data ->_legController->data[0].q(2) <<" "<< _data ->_legController->data[1].q(2) <<" "<<_data ->_legController->data[2].q(2)<<" "<<_data ->_legController->data[3].q(2)<<" "
        <<_data ->_legController->data[0].qd(0) <<" "<<_data ->_legController->data[1].qd(0) <<" "<<_data ->_legController->data[2].qd(0)<<" "<<_data ->_legController->data[3].qd(0)<< " "
        <<_data ->_legController->data[0].qd(1) <<" "<< _data ->_legController->data[1].qd(1) <<" "<<_data ->_legController->data[2].qd(1)<<" "<<_data ->_legController->data[3].qd(1) << " "
        <<_data ->_legController->data[0].qd(2) <<" "<< _data ->_legController->data[1].qd(2) <<" "<<_data ->_legController->data[2].qd(2)<<" "<<_data ->_legController->data[3].qd(2)<<std::endl;
    
        joint_des << _data ->_legController->commands[0].qDes(0) <<" "<< _data ->_legController->commands[1].qDes(0) <<" "<<_data ->_legController->commands[2].qDes(0)<<" "<<_data ->_legController->commands[3].qDes(0)<<" "
    	  << _data ->_legController->commands[0].qDes(1) <<" "<< _data ->_legController->commands[1].qDes(1) <<" "<<_data ->_legController->commands[2].qDes(1)<<" "<<_data ->_legController->commands[3].qDes(1)<<" "
    	  << _data ->_legController->commands[0].qDes(2) <<" "<< _data ->_legController->commands[1].qDes(2) <<" "<<_data ->_legController->commands[2].qDes(2)<<" "<<_data ->_legController->commands[3].qDes(2)<<" "
    	  << _data ->_legController->commands[0].qdDes(0) <<" "<< _data ->_legController->commands[1].qdDes(0) <<" "<<_data ->_legController->commands[2].qdDes(0)<<" "<<_data ->_legController->commands[3].qdDes(0)<<" "
    	  << _data ->_legController->commands[0].qdDes(1) <<" "<< _data ->_legController->commands[1].qdDes(1) <<" "<<_data ->_legController->commands[2].qdDes(1)<<" "<<_data ->_legController->commands[3].qdDes(1)<<" "
    	  << _data ->_legController->commands[0].qdDes(2) <<" "<< _data ->_legController->commands[1].qdDes(2) <<" "<<_data ->_legController->commands[2].qdDes(2)<<" "<<_data ->_legController->commands[3].qdDes(2)<<" "
    	  << 0 <<" "<< 0 <<" "<<0<<" "<<0<<" "
    	  << -tauDes[0][idx] <<" "<< -tauDes[0][idx] <<" "<<-tauDes[2][idx] <<" "<<-tauDes[2][idx]<<" "
    	  << -tauDes[1][idx] <<" "<< -tauDes[0][idx] <<" "<<-tauDes[3][idx]<<" "<<-tauDes[3][idx]<<std::endl;

        // // FR, FL, RR, RL
        // foot_act<< _data ->_legController ->data[0].p(0)<< " " << _data ->_legController ->data[0].p(1)<<" "<< _data ->_legController ->data[0].p(2)<<" "
        // <<_data ->_legController ->data[1].p(0)<< " " <<_data ->_legController ->data[1].p(1)<<" "<< _data ->_legController ->data[1].p(2)<<" "
        // <<_data ->_legController ->data[2].p(0)<< " " << _data ->_legController ->data[2].p(1)<<" "<< _data ->_legController ->data[2].p(2)<<" "
        // <<_data ->_legController ->data[3].p(0)<< " " << _data ->_legController ->data[3].p(1)<<" "<< _data ->_legController ->data[3].p(2)<<" "
        // <<_data ->_legController ->data[0].v(0)<< " " << _data ->_legController ->data[0].v(1)<<" "<< _data ->_legController ->data[0].v(2)<<" "
        // <<_data ->_legController ->data[1].v(0)<< " " << _data ->_legController ->data[1].v(1)<<" "<< _data ->_legController ->data[1].v(2)<<" "
        // <<_data ->_legController ->data[2].v(0)<< " " << _data ->_legController ->data[2].v(1)<<" "<< _data ->_legController ->data[2].v(2)<<" "
        // <<_data ->_legController ->data[3].v(0)<< " " << _data ->_legController ->data[3].v(1)<<" "<< _data ->_legController ->data[3].v(2)<<std::endl;
        
        // // FR, FL, RR, RL
        // foot_des<< _data ->_legController ->commands[0].pDes(0)<< " " << _data ->_legController ->commands[0].pDes(1)<<" "<< _data ->_legController ->commands[0].pDes(2)<<" "
        // <<_data ->_legController ->commands[1].pDes(0)<< " " << _data ->_legController ->commands[1].pDes(1)<<" "<< _data ->_legController ->commands[1].pDes(2)<<" "
        // <<_data ->_legController ->commands[2].pDes(0)<< " " << _data ->_legController ->commands[2].pDes(1)<<" "<< _data ->_legController ->commands[2].pDes(2)<<" "
        // <<_data ->_legController ->commands[3].pDes(0)<< " " << _data ->_legController ->commands[3].pDes(1)<<" "<< _data ->_legController ->commands[3].pDes(2)<<" "
        // <<_data ->_legController ->commands[0].vDes(0)<< " " << _data ->_legController ->commands[0].vDes(1)<<" "<< _data ->_legController ->commands[0].vDes(2)<<" "
        // <<_data ->_legController ->commands[1].vDes(0)<< " " << _data ->_legController ->commands[1].vDes(1)<<" "<< _data ->_legController ->commands[1].vDes(2)<<" "
        // <<_data ->_legController ->commands[2].vDes(0)<< " " << _data ->_legController ->commands[2].vDes(1)<<" "<< _data ->_legController ->commands[2].vDes(2)<<" "
        // <<_data ->_legController ->commands[3].vDes(0)<< " " << _data ->_legController ->commands[3].vDes(1)<<" "<< _data ->_legController ->commands[3].vDes(2)<<std::endl;
        
        optitrack << OptiTrack_data[0] << "  " << OptiTrack_data[1] << " " << OptiTrack_data[2] << " " << OptiTrack_data[3] << " " << OptiTrack_data[4] << " " << OptiTrack_data[5] << " " << OptiTrack_data[6] << std::endl;

        pos << _data->_stateEstimator->getResult().vWorld(0) << " " << _data->_stateEstimator->getResult().vWorld(1) << " " <<  _data->_stateEstimator->getResult().vWorld(2) << " " 
        <<_data->_stateEstimator->getResult().position(0) << " " << _data->_stateEstimator->getResult().position(1) << " " << _data->_stateEstimator->getResult().position(2) << " " 
        << _data->_stateEstimator->getResult().rpy(0) << " " << _data->_stateEstimator->getResult().rpy(1) << " " << _data->_stateEstimator->getResult().rpy(2) << " " 
        << _data->_legController->commands[0].force_MPC(0) << " " <<  _data->_legController->commands[1].force_MPC(0) << " " <<  _data->_legController->commands[2].force_MPC(0) << " " <<  _data->_legController->commands[3].force_MPC(0) << " " 
        << _data->_legController->commands[0].force_MPC(1) << " " <<  _data->_legController->commands[1].force_MPC(1) << " " <<  _data->_legController->commands[2].force_MPC(1) << " " <<  _data->_legController->commands[3].force_MPC(1) << " " 
        << _data->_legController->commands[0].force_MPC(2) << " " <<  _data->_legController->commands[1].force_MPC(2) << " " <<  _data->_legController->commands[2].force_MPC(2) << " " <<  _data->_legController->commands[3].force_MPC(2) << " " 
        << -FDes[0][idx]/2 <<" "<< -FDes[1][idx]/2 <<" "<< -FDes[2][idx]/2 <<" "<< -FDes[3][idx]/2 << " "
        << _data->_legController->commands[0].feedforwardForce(0) << " " <<  _data->_legController->commands[1].feedforwardForce(0) << " " <<  _data->_legController->commands[2].feedforwardForce(0) << " " <<  _data->_legController->commands[3].feedforwardForce(0) << " " 
        << _data->_legController->commands[0].feedforwardForce(1) << " " <<  _data->_legController->commands[1].feedforwardForce(1) << " " <<  _data->_legController->commands[2].feedforwardForce(1) << " " <<  _data->_legController->commands[3].feedforwardForce(1) << " " 
        << _data->_legController->commands[0].feedforwardForce(2) << " " <<  _data->_legController->commands[1].feedforwardForce(2) << " " <<  _data->_legController->commands[2].feedforwardForce(2) << " " <<  _data->_legController->commands[3].feedforwardForce(2) << " " 
        << _data->_stateEstimator->getResult().contactEstimate(0) << " " << _data->_stateEstimator->getResult().contactEstimate(1) << " " << _data->_stateEstimator->getResult().contactEstimate(2) << " " << _data->_stateEstimator->getResult().contactEstimate(3)<<" "
        << lowState.footForce[0] << " " << lowState.footForce[1] <<" "<< lowState.footForce[2]<<" " << lowState.footForce[3]<< " " << t_convert << std::endl;
       
        data_f << _data->_stateEstimator->getResult().position(0) << " " << _data->_stateEstimator->getResult().position(2) << " " << _data->_stateEstimator->getResult().rpy(1) << " "
        << _data->_stateEstimator->getResult().vWorld(0) << " " << _data->_stateEstimator->getResult().vWorld(2) << " " << _data->_stateEstimator->getResult().omegaWorld(1) << " " << 9.81 << " "
        << pFeet[0] << " " << pFeet[2] << " " << pFeet[6] <<" " << pFeet[8] << " " 
        << fMPC[0] << " " << fMPC[1] << " "<< fMPC[2]<< " " << fMPC[3]<< std::endl;

        data_pd << f_PD[0] << " " << f_PD[1] << " "<< f_PD[2]<< " " << f_PD[3]<< std::endl;    

        pfeet_optitrack << pFeet_Optrack[0] << " " << pFeet_Optrack[2] << " " << pFeet_Optrack[6] << " " << pFeet_Optrack[8] << std::endl;

    }



    // _data->_legController->updateCommand();
    if (motiontime< startjump){
      _data->_legController->updateCommand();

    }

    if(motiontime>=startjump && motiontime<startjump+N_TrajRef-pose_time){
      _data->_legController->updateCommand_MDC();
    }

    if(motiontime >= startjump+N_TrajRef-pose_time){
      _data->_legController->updateCommand();

    }


    //motiontime++;
 
    // control.PowerProtect(lowCmd, lowState, 10);
}

void FSM_State::QPstand(){

    // reset forces and steps to 0
    Recv();
    //std::cout << "start stand" << std::endl;
    footFeedForwardForces = Mat34<double>::Zero();
    footstepLocations = Mat34<double>::Zero();
    double minForce = 5;
    double maxForce = 250;
    double contactStateScheduled[4] = {1, 1, 1, 1};
    
    double minForces[4] = {minForce, minForce, minForce, minForce};
    double maxForces[4] = {maxForce, maxForce, maxForce, maxForce};
    Vec4<double> contactphase(0.5,0.5,0.5,0.5); // default contact phase
    //Timer time;
    //time.start();
    if(!initiated){
       _data->_stateEstimator->setContactPhase(contactphase);
    }
    //double COM_weights_stance[3] = {50, 50, 50};
    //double Base_weights_stance[3] = {500, 150, 30};// for QP locomotion
    double COM_weights_stance[3] = {5, 5, 5};
    double Base_weights_stance[3] = {40, 20, 10};
    double pFeet[12], p_act[3], v_act[3], O_err[3], rpy[3],v_des[3], p_des[3],
      omegaDes[3];
    double se_xfb[13];
    double kpCOM[3], kdCOM[3], kpBase[3], kdBase[3];
    double b_control[6];
    
    for(int i = 0; i < 3; i++){
      p_des[i] = 0;
      v_des[i] = 0;
      omegaDes[i] = 0;
      rpy[i] = 0;
    
    }
    //double v_des[3] = {0, 0, 0};
     p_des[2] = 0.3; // standup height for A1
    
     //std::cout << ros::Time::now() << std::endl;
      // rate.reset();
      //std::cout << "get leg state" << std::endl;
       _data->_desiredStateCommand->convertToStateCommands();
       _data->_legController->updateData();
       //std::cout << "get state" << std::endl;
       _data->_stateEstimator->run();

       if(motiontime > 1000){
         initiated = true;
       }
  
    /*======================== Locomotion ====================*/
   if(_data->_desiredStateCommand->data.mode == 2){
      
    //  if(firstwalk){
    //    for(int i = 0; i < 3; i++){
    //     init_walk_pos(i) = _data->_stateEstimator->getResult().position(i);
    //    }

    //    firstwalk = false;
    //  }
    
      // Vec3<double> v_des(0.0 , 0.0, 0);
      // double yaw_rate = 0.0;
      Cmpc.setGaitNum(2);

      // double x_act = _data->_stateEstimator->getResult().position(0);
      // double y_act = _data->_stateEstimator->getResult().position(1);

      // v_des[0] = -0.05 * x_act;
      // v_des[1] = -0.05 * y_act;

      // for(int i = 0; i < 4; i++){
      // std::cout << "leg " << i << "Contact phase:  " << _data->_stateEstimator->getResult().contactEstimate(i) << std::endl;
      // }
      // if(motiontime > 5000 && motiontime < 6000){
        // yaw_rate = -0.3
      // }

      // if(motiontime > 6000 && motiontime < 7000){
      //   // v_des[0] = 0;
      //   // v_des[1] = 0.2;
      // }
      // if(motiontime > 7000){
      //   // v_des[1] = 0;
      // }
   
    Cmpc.run(*_data); // MPC Controller
    // QPloco.run(*_data); // QPlocomotion
 
    runQP = false;
    firstrunQP = true;
   
   }
  //  else{
  //    runQP = true;
    
  //    for(int i = 0; i < 4; i++){
  //    _data->_legController->commands[i].kdCartesian << 0, 0, 0,
  //                                                      0, 0, 0,
  //                                                      0, 0, 0;
  //     _data->_legController->commands[i].kpCartesian << 0, 0, 0,
  //                                                      0, 0, 0,
  //                                                      0, 0, 0;                                                  
  //    }
  //  }
    
    pos << _data->_stateEstimator->getResult().vWorld(0) << " " << _data->_stateEstimator->getResult().vWorld(1) << 
    " " <<  _data->_stateEstimator->getResult().vWorld(2) << " " <<_data->_stateEstimator->getResult().position(0) << " " << _data->_stateEstimator->getResult().position(1) << " "
    << _data->_stateEstimator->getResult().position(2) << " " << _data->_stateEstimator->getResult().rpy(0) << " " << _data->_stateEstimator->getResult().rpy(1) << " " 
    << _data->_stateEstimator->getResult().rpy(2) << " " << _data->_legController->commands[0].feedforwardForce(2) << " " <<  _data->_legController->commands[1].feedforwardForce(2) << " "
    <<  _data->_legController->commands[2].feedforwardForce(2) << " " <<  _data->_legController->commands[3].feedforwardForce(2) << 
    " " << _data->_stateEstimator->getResult().contactEstimate(0) << " " << _data->_stateEstimator->getResult().contactEstimate(1) 
    << " " << _data->_stateEstimator->getResult().contactEstimate(2) << " " << _data->_stateEstimator->getResult().contactEstimate(3) <<std::endl;

    
        
    /* =================================================================*/
    if(_data->_desiredStateCommand->data.mode == 1  && initiated){
       _data->_stateEstimator->setContactPhase(contactphase);
      if(firstrunQP){
        // rpy[2] = _data->_stateEstimator->getResult().rpy(2);
        init_yaw = _data->_stateEstimator->getResult().rpy(2);
        firstrunQP = false;
      }
      // set rpy command
      rpy[0] = 0;
      rpy[1] = 0;
      rpy[2] = init_yaw;

      for (int i = 0; i < 4; i++) {
        se_xfb[i] = _data->_stateEstimator->getResult().orientation(i);
      }

      for (int i = 0; i < 3; i++) {
        // rpy[i] = 0;
        //rpy[i] = _data->_stateEstimator->getResult().rpy(i);
        //p_des[i] = _data->_stateEstimator->getResult().position(i);
        p_act[i] = _data->_stateEstimator->getResult().position(i);
        
        v_act[i] = _data->_stateEstimator->getResult().vWorld(i);
        // v_des[i] = 0;
        //v_des[2] = 0.005;

        se_xfb[4 + i] = _data->_stateEstimator->getResult().position(i);
        se_xfb[7 + i] = _data->_stateEstimator->getResult().omegaBody(i);
        se_xfb[10 + i] = _data->_stateEstimator->getResult().vWorld(i);
        
    // Set the translational and orientation gains
      // Set the translational and orientation gains
        kpCOM[i] = 25;   //_data->controlParameters->kpCOM(i);
        kdCOM[i] = 5;     //_data->controlParameters->kdCOM(i);
        kpBase[i] = 200;     //_data->controlParameters->kpBase(i);
        kdBase[i] = 10; //  _data->controlParameters->kdBase(i);
        }
      kpCOM[2] = 55;
      // kdCOM[2] = 20;
      kpBase[0] = 600;
      kdBase[0] = 5;
      kpBase[1] = 400;

    //Vec3<double> pFeetVec;
    Vec3<double> pFeetVecCOM;

    // Get the foot locations relative to COM
      for (int leg = 0; leg < 4; leg++) {
        // computeLegJacobianAndPosition(&_data->_quadruped, _data->_legController->data[leg].q,
        //                           (Mat3<double>*)nullptr, &pFeetVec, leg);
        //pFeetVecCOM = _data->_stateEstimator->getResult().rBody.transpose() *
                  //(_data->_quadruped->getHipLocation(leg) + pFeetVec);

        pFeetVecCOM =  _data->_stateEstimator->getResult().rBody.transpose() *
        (_data->_quadruped->getHipLocation(leg) + _data->_legController->data[leg].p);


        pFeet[leg * 3] = pFeetVecCOM[0];
        pFeet[leg * 3 + 1] = pFeetVecCOM[1];
        pFeet[leg * 3 + 2] = pFeetVecCOM[2];
        //std::cout << "pFeet" << leg << std::endl;
      }

      p_des[0] = p_act[0] + (pFeet[0] + pFeet[3] + pFeet[6] + pFeet[9])/4.0;
      p_des[1] = p_act[1] + (pFeet[1] + pFeet[4] + pFeet[7] + pFeet[10])/4.0;
    //  myfile << "\n";
     // std::cout << j << std::endl;
     //std::cout << "run QP" << std::endl;
    balanceController.set_alpha_control(0.01);
    balanceController.set_friction(0.2);
    balanceController.set_mass(_data->_quadruped->mass);
    balanceController.set_wrench_weights(COM_weights_stance, Base_weights_stance);
    balanceController.set_PDgains(kpCOM, kdCOM, kpBase, kdBase);
    balanceController.set_desiredTrajectoryData(rpy, p_des, omegaDes, v_des);
    balanceController.SetContactData(contactStateScheduled, minForces, maxForces);
    balanceController.updateProblemData(se_xfb, pFeet, p_des, p_act, v_des, v_act,
                                      O_err, _data->_stateEstimator->getResult().rpy(2));
   // balanceController.print_QPData();
    double fOpt[12];
    balanceController.solveQP_nonThreaded(fOpt);
    //balanceController.get_b_matrix(b_control);
    //b_des << b_control[2] << "\n";

  // Publish the results over ROS
  // balanceController.publish_data_lcm();

  // Copy the results to the feed forward forces
    
     //_data->_stateEstimator->run();


    for (int leg = 0; leg < 4; leg++) {
        footFeedForwardForces.col(leg) << fOpt[leg * 3], fOpt[leg * 3 + 1],
        fOpt[leg * 3 + 2]; // force in world frame, need to convert to body frame

        _data->_legController->commands[leg].feedforwardForce = footFeedForwardForces.col(leg);
        //_data->_stateEstimator->getResult().rBody.transpose() * footFeedForwardForces.col(leg); 
       // QP << _data->_legController->commands[leg].feedforwardForce[2] << " ";
    }
  
    //std::cout << j << std::endl;
    //QP << "\n";
    
    
    //if(p_act[2] < 0.25){
     //  std::cout << "force" << std::endl;
    //std::cout << footFeedForwardForces << std::endl;
    //_data->_legController->updateCommand();

    // plots
    // if(counter>500){
    //  for(int i = 0; i < 4; i++){
    //    QP << _data->_legController->commands[i].feedforwardForce[2] << " ";
    //    //b_des << _data->_legController->commands[i].pDes[2] << " "; // foot pos
       
    //   }
    //   b_des << _data->_legController->commands[1].pDes[2] << " ";
    //   b_des << _data->_legController->data[1].p[2] << "\n ";
      
    //   for(int i =0; i<3; i++){
    //     //myfile << _data->_stateEstimator->getResult().rpy(i);
    //     //myfile << " " ;
    //      myfile << _data->_legController->commands[1].pDes[i] - _data->_legController->data[1].p[i] << " ";
    //   }
    //  myfile << "\n";
    //   z_pos << _data->_stateEstimator->getResult().position(2) << "\n";
    //  QP << "\n";
    //  //b_des << "\n";
    // }
    }
    // if(motiontime > 5000){
    // pos << _data->_stateEstimator->getResult().vWorld(0) << " " << _data->_stateEstimator->getResult().vWorld(1) << 
    // " " <<  _data->_stateEstimator->getResult().vWorld(2) << " " <<_data->_stateEstimator->getResult().position(0) << " " << _data->_stateEstimator->getResult().position(1) << " "
    // << _data->_stateEstimator->getResult().position(2) <<std::endl;
    // }
    // _data->_legController->zeroCommand();
    _data->_legController->updateCommand();
  
    //motiontime++;
 
    // control.PowerProtect(lowCmd, lowState, 10);
}

void FSM_State::Landing_test(){

    // reset forces and steps to 0
    Recv();
    footFeedForwardForces = Mat34<double>::Zero();
    double minForce = 5;
    double maxForce = 500;
    double contactStateScheduled[4] = {1, 1, 1, 1};
    
    double minForces[4] = {minForce, minForce, minForce, minForce};
    double maxForces[4] = {maxForce, maxForce, maxForce, maxForce};
    Vec4<double> contactphase(0.5,0.5,0.5,0.5); // default contact phase
    //Timer time;
    //time.start();
    if(!initiated){
       _data->_stateEstimator->setContactPhase(contactphase);
    }
    //double COM_weights_stance[3] = {50, 50, 50};
    //double Base_weights_stance[3] = {500, 150, 30};// for QP locomotion
    double COM_weights_stance[3] = {5, 5, 10}; // 50,50,50,
    double Base_weights_stance[3] = {10, 10, 20};// 40,20,10
    double pFeet[12], p_act[3], v_act[3], O_err[3], rpy[3],v_des[3], p_des[3],
      omegaDes[3];
    double se_xfb[13];
    double kpCOM[3], kdCOM[3], kpBase[3], kdBase[3];
    double b_control[6];
    
    for(int i = 0; i < 3; i++){
      p_des[i] = 0;
      v_des[i] = 0;
      omegaDes[i] = 0;
      rpy[i] = 0;
    
    }
    //double v_des[3] = {0, 0, 0};
     //p_des[2] = 0.2347; // initial standup height for A1
     p_des[2] = 0.2;
     //p_des[2] = 0.195;
    
     //std::cout << ros::Time::now() << std::endl;
      // rate.reset();
      //std::cout << "get leg state" << std::endl;
       _data->_desiredStateCommand->convertToStateCommands();
       _data->_legController->updateData();
       //std::cout << "get state" << std::endl;
       _data->_stateEstimator->run();

       if(motiontime > 1000){
         initiated = true;
       }

    /*======================== Locomotion ====================*/
    if(_data->_desiredStateCommand->data.mode == 2){ // When press button start on controller
      runQP = false;
      firstrunQP = true;
    }
    
    double Kp = 250;
    double Kd = 5; 
    double Hip_Kp = 250;
    double Hip_Kd =5;
    double Kp_Ca= 400;
    double Kd_Ca =25;
    double Kp_l= 5;
    double Kd_l=1;
    int idx = 0; // data index
    int startjump = 3000; // index start jumping
    //int data_size=1110; // jump left down
    int data_size=1310; // jump left down
    int pose_time=250;
    int idx_pose=data_size-pose_time;
    // double init_Pos[12] = {0, 1.2566, -2.355, 0, 1.2566, -2.355, 0, 1.2566, -2.355, 0, 1.2566, -2.355}; // pose 0
    double init_Pos[12] = {0, 1.1310, -2.1206, 0, 1.1310, -2.1206, 0, 1.1310, -2.1206, 0, 1.1310, -2.1206}; // pose 0, 64, -121
    double currentPos[12];
    //std::cout << "Start jumping" << std::endl;
    double FR=0; double FL=0; double RR=0; double RL=0; // contact state
    double threshold=8;
        

    if (motiontime <startjump){  // Initialization
      runQP=false;
      std::cout << "FRfootCallback: " << lowState.footForce[0] << std::endl;
      std::cout << "FLfootCallback: " << lowState.footForce[1] << std::endl;
      std::cout << "RRfootCallback: " << lowState.footForce[2] << std::endl;
      std::cout << "RLfootCallback: " << lowState.footForce[3] << std::endl;
      //double contactStateScheduled[4]={1,1,1,1}; // all legs are stance legs
      for(int i = 0; i < 4; i++){
            _data->_legController->commands[i].kpJoint = Vec3<double>(Hip_Kp,Kp,Kp).asDiagonal();
            _data->_legController->commands[i].kdJoint = Vec3<double>(Hip_Kd,Kd,Kd).asDiagonal();
            _data->_legController->commands[i].kpCartesian << Mat3<double>::Zero();
            _data->_legController->commands[i].kdCartesian << Mat3<double>::Zero();
      }

      percent = (double) motiontime/ startjump; 
      std ::cout << "percent: " << percent << std::endl; 
      for (int i=0; i<4; i++){ 
               for(int j=0; j<3; j++){ 
                 currentPos[3*i+j] = _data->_legController->data[i].q(j); 
                 _data->_legController->commands[i].qDes(j) = currentPos[3*i+j]*(1-percent) + init_Pos[3*i+j]*percent; 
               } 
            } 
      }

      if (motiontime >=startjump){ // run QP

          // rotate leg

          _data->_legController->commands[0].tau << 0, 0, 0;// front right leg
          _data->_legController->commands[1].tau << 0, 0, 0; // front left leg
          _data->_legController->commands[2].tau << 0, 0, 0;// rear right leg
          _data->_legController->commands[3].tau << 0, 0, 0; // rear left leg

          _data->_legController->commands[0].qDes << -0.3, 1.1310, -2.1206; // front right leg
          _data->_legController->commands[1].qDes << 0.3, 1.1310, -2.1206; // front left leg
          _data->_legController->commands[2].qDes << -0.3, 1.1310, -2.1206;// rear right leg
          _data->_legController->commands[3].qDes << 0.3, 1.1310, -2.1206; // rear left leg

          _data->_legController->commands[0].qdDes << 0, 0, 0; // front right leg
          _data->_legController->commands[1].qdDes << 0, 0, 0; // front left leg
          _data->_legController->commands[2].qdDes << 0, 0, 0;// rear right leg
          _data->_legController->commands[3].qdDes << 0, 0, 0; // rear left leg

          for (int i=0; i<4; i++){
              _data->_legController->commands[i].kpCartesian << Mat3<double>::Zero();
              _data->_legController->commands[i].kdCartesian << Mat3<double>::Zero();

          }

          // Get contact state
          FR=lowState.footForce[0];
          FL=lowState.footForce[1];
          RR=lowState.footForce[2];
          RL=lowState.footForce[3];
          if (FR>=threshold){
            FR=1;
            // _data->_legController->commands[0].kpJoint << Mat3<double>::Zero();
            // _data->_legController->commands[0].kdJoint << Mat3<double>::Zero();
            _data->_legController->commands[0].kpJoint << Kp_l, 0, 0,
                                                          0, Kp_l, 0,
                                                          0, 0, Kp_l;
            _data->_legController->commands[0].kdJoint << Kd_l, 0, 0,
                                                          0, Kd_l, 0,
                                                          0, 0, Kd_l;
            // _data->_legController->commands[0].kpCartesian << Mat3<double>::Zero();
            // _data->_legController->commands[0].kdCartesian << Mat3<double>::Zero();
            // _data->_legController->commands[0].tau << Vec3<double>::Zero();// front right leg
          }
          else{
            FR=0;
          }

          if (FL>=threshold){
            FL=1;
            //_data->_legController->commands[1].kpJoint << Mat3<double>::Zero();
            //_data->_legController->commands[1].kdJoint << Mat3<double>::Zero();
            _data->_legController->commands[1].kpJoint << Kp_l, 0, 0,
                                                          0, Kp_l, 0,
                                                          0, 0, Kp_l;
            _data->_legController->commands[1].kdJoint << Kd_l, 0, 0,
                                                          0, Kd_l, 0,
                                                          0, 0, Kd_l;
            // _data->_legController->commands[1].kpCartesian << Mat3<double>::Zero();
            // _data->_legController->commands[1].kdCartesian << Mat3<double>::Zero();
            // _data->_legController->commands[1].tau << Vec3<double>::Zero();// front right leg

          }
          else{
            FL=0;
          }

          if (RL>=threshold){
            RL=1;
            // _data->_legController->commands[3].kpJoint << Mat3<double>::Zero();
            // _data->_legController->commands[3].kdJoint << Mat3<double>::Zero();
            _data->_legController->commands[3].kpJoint << Kp_l, 0, 0,
                                                          0, Kp_l, 0,
                                                          0, 0, Kp_l;
            _data->_legController->commands[3].kdJoint << Kd_l, 0, 0,
                                                          0, Kd_l, 0,
                                                          0, 0, Kd_l;

            // _data->_legController->commands[3].kpCartesian << Mat3<double>::Zero();
            // _data->_legController->commands[3].kdCartesian << Mat3<double>::Zero();
            // _data->_legController->commands[3].tau << Vec3<double>::Zero();// front right leg
          }
          else{
            RL=0;
          }

          if (RR>=threshold){
            RR=1;
            // _data->_legController->commands[2].kpJoint << Mat3<double>::Zero();
            // _data->_legController->commands[2].kdJoint << Mat3<double>::Zero();
            _data->_legController->commands[2].kpJoint << Kp_l, 0, 0,
                                                          0, Kp_l, 0,
                                                          0, 0, Kp_l;
            _data->_legController->commands[2].kdJoint << Kd_l, 0, 0,
                                                          0, Kd_l, 0,
                                                          0, 0, Kd_l;
            // _data->_legController->commands[2].kpCartesian << Mat3<double>::Zero();
            // _data->_legController->commands[2].kdCartesian << Mat3<double>::Zero();
            // _data->_legController->commands[2].tau << Vec3<double>::Zero();// front right leg

          }
          else{
            RR=0;
          }

          double contactStateScheduled[4]={FR,FL,RR,RL};
          std::cout << "contactStateScheduled: " << contactStateScheduled[0] << "," << contactStateScheduled[1]<<"," << contactStateScheduled[2]<< "," << contactStateScheduled[3]<< std::endl;
          runQP=true;
      }
      
        
    /* =================================================================*/

    
    if(runQP){
       _data->_stateEstimator->setContactPhase(contactphase);
      if(firstrunQP){
        // rpy[2] = _data->_stateEstimator->getResult().rpy(2);
        init_yaw = _data->_stateEstimator->getResult().rpy(2);
        firstrunQP = false;
      }
      // set rpy command
      rpy[0] = 0;
      rpy[1] = 0;
      rpy[2] = init_yaw;

      for (int i = 0; i < 4; i++) {
        se_xfb[i] = _data->_stateEstimator->getResult().orientation(i);
      }

      for (int i = 0; i < 3; i++) {
        // rpy[i] = 0;
        //rpy[i] = _data->_stateEstimator->getResult().rpy(i);
        //p_des[i] = _data->_stateEstimator->getResult().position(i);
        p_act[i] = _data->_stateEstimator->getResult().position(i);
        
        v_act[i] = _data->_stateEstimator->getResult().vWorld(i);
        // v_des[i] = 0;
        //v_des[2] = 0.005;

        se_xfb[4 + i] = _data->_stateEstimator->getResult().position(i);
        se_xfb[7 + i] = _data->_stateEstimator->getResult().omegaBody(i);
        se_xfb[10 + i] = _data->_stateEstimator->getResult().vWorld(i);
        
    // Set the translational and orientation gains
      // Set the translational and orientation gains
        kpCOM[i] = 30;   //_data->controlParameters->kpCOM(i);
        kdCOM[i] = 10;     //_data->controlParameters->kdCOM(i);
        kpBase[i] = 80;     //_data->controlParameters->kpBase(i);
        kdBase[i] = 20; //  _data->controlParameters->kdBase(i);
        }
      kpCOM[2] = 50;
      kdCOM[2] = 20;
      kpBase[0] = 300;
      //kdBase[0] = 5;
      kpBase[1] = 200;

    //Vec3<double> pFeetVec;
    Vec3<double> pFeetVecCOM;

    // Get the foot locations relative to COM
      for (int leg = 0; leg < 4; leg++) {
        // computeLegJacobianAndPosition(&_data->_quadruped, _data->_legController->data[leg].q,
        //                           (Mat3<double>*)nullptr, &pFeetVec, leg);
        //pFeetVecCOM = _data->_stateEstimator->getResult().rBody.transpose() *
                  //(_data->_quadruped->getHipLocation(leg) + pFeetVec);

        pFeetVecCOM =  _data->_stateEstimator->getResult().rBody.transpose() *
        (_data->_quadruped->getHipLocation(leg) + _data->_legController->data[leg].p);


        pFeet[leg * 3] = pFeetVecCOM[0];
        pFeet[leg * 3 + 1] = pFeetVecCOM[1];
        pFeet[leg * 3 + 2] = pFeetVecCOM[2];
        //std::cout << "pFeet" << leg << std::endl;
      }

      p_des[0] = p_act[0] + (pFeet[0] + pFeet[3] + pFeet[6] + pFeet[9])/4.0;
      p_des[1] = p_act[1] + (pFeet[1] + pFeet[4] + pFeet[7] + pFeet[10])/4.0;
    //  myfile << "\n";
     // std::cout << j << std::endl;
     //std::cout << "run QP" << std::endl;
    balanceController.set_alpha_control(0.01);
    balanceController.set_friction(0.2);
    balanceController.set_mass(_data->_quadruped->mass);
    balanceController.set_wrench_weights(COM_weights_stance, Base_weights_stance);
    balanceController.set_PDgains(kpCOM, kdCOM, kpBase, kdBase);
    balanceController.set_desiredTrajectoryData(rpy, p_des, omegaDes, v_des);
    balanceController.SetContactData(contactStateScheduled, minForces, maxForces);
    balanceController.updateProblemData(se_xfb, pFeet, p_des, p_act, v_des, v_act,
                                      O_err, _data->_stateEstimator->getResult().rpy(2));
   // balanceController.print_QPData();
    double fOpt[12];
    balanceController.solveQP_nonThreaded(fOpt);
    //balanceController.get_b_matrix(b_control);
    //b_des << b_control[2] << "\n";

  // Publish the results over ROS
  // balanceController.publish_data_lcm();

  // Copy the results to the feed forward forces
    
     //_data->_stateEstimator->run();


    for (int leg = 0; leg < 4; leg++) {
        footFeedForwardForces.col(leg) << fOpt[leg * 3], fOpt[leg * 3 + 1],
        fOpt[leg * 3 + 2]; // force in world frame, need to convert to body frame

        _data->_legController->commands[leg].feedforwardForce = footFeedForwardForces.col(leg);
        //_data->_stateEstimator->getResult().rBody.transpose() * footFeedForwardForces.col(leg); 
       // QP << _data->_legController->commands[leg].feedforwardForce[2] << " ";
    }
  
    //std::cout << j << std::endl;
    //QP << "\n";
    
    
    //if(p_act[2] < 0.25){
     //  std::cout << "for std::endl;
    //std::cout << footFeedForwardForces << std::endl;
    //_data->_legController->updateCommand();

    // plots
    // if(counter>500){
    //  for(int i = 0; i < 4; i++){
    //    QP << _data->_legController->commands[i].feedforwardForce[2] << " ";
    //    //b_des << _data->_legController->commands[i].pDes[2] << " "; // foot pos
       
    //   }
    //   b_des << _data->_legController->commands[1].pDes[2] << " ";
    //   b_des << _data->_legController->data[1].p[2] << "\n ";
      
    //   for(int i =0; i<3; i++){
    //     //myfile << _data->_stateEstimator->getResult().rpy(i);
    //     //myfile << " " ;
    //      myfile << _data->_legController->commands[1].pDes[i] - _data->_legController->data[1].p[i] << " ";
    //   }
    //  myfile << "\n";
    //   z_pos << _data->_stateEstimator->getResult().position(2) << "\n";
    //  QP << "\n";
    //  //b_des << "\n";
    // }
    }
    
    // send command
     _data->_legController->updateCommand_Jump3D_4base();



    // export data
    if (motiontime >=startjump && motiontime <= startjump+20000){

        pos << _data->_stateEstimator->getResult().vWorld(0) << " " << _data->_stateEstimator->getResult().vWorld(1) << " " <<  _data->_stateEstimator->getResult().vWorld(2) << " " 
        <<_data->_stateEstimator->getResult().position(0) << " " << _data->_stateEstimator->getResult().position(1) << " " << _data->_stateEstimator->getResult().position(2) << " " 
        << _data->_stateEstimator->getResult().rpy(0) << " " << _data->_stateEstimator->getResult().rpy(1) << " " << _data->_stateEstimator->getResult().rpy(2) << " " 
        << _data->_legController->commands[0].feedforwardForce(2) << " " <<  _data->_legController->commands[1].feedforwardForce(2) << " " <<  _data->_legController->commands[2].feedforwardForce(2) << " " <<  _data->_legController->commands[3].feedforwardForce(2) << " " 
        << _data->_stateEstimator->getResult().contactEstimate(0) << " " << _data->_stateEstimator->getResult().contactEstimate(1) << " " << _data->_stateEstimator->getResult().contactEstimate(2) << " " << _data->_stateEstimator->getResult().contactEstimate(3)<<" "
        << lowState.footForce[0] << " " << lowState.footForce[1] <<" "<< lowState.footForce[2]<<" " << lowState.footForce[3]<< std::endl;
    }
    // _data->_legController->zeroCommand();
    //_data->_legController->updateCommand_Jump3D_4base();


  
    //motiontime++;
 
    // control.PowerProtect(lowCmd, lowState, 10);

    std::cout << "-----------------------------------------"<< std::endl;
}


void FSM_State::runQPloco()
{
  Recv();
  // reset forces and steps to 0
  footFeedForwardForces = Mat34<double>::Zero();
  footstepLocations = Mat34<double>::Zero();
  double minForce = 5;
  double maxForce = 500;
  double contactStateScheduled[4] = {1, 1, 1, 1};

  double minForces[4] = {minForce, minForce, minForce, minForce};
  double maxForces[4] = {maxForce, maxForce, maxForce, maxForce};
  Vec4<double> contactphase(0.5,0.5,0.5,0.5); // default contact phase
  //Timer time;
  //time.start();
  //double COM_weights_stance[3] = {50, 50, 50};
  //double Base_weights_stance[3] = {500, 150, 30};// for QP locomotion
  double COM_weights_stance[3] = {10, 10, 10};
  double Base_weights_stance[3] = {100, 25, 20};
  double pFeet[12], p_act[3], v_act[3], O_err[3], rpy[3],v_des[3], p_des[3],
      omegaDes[3];
  double se_xfb[13];
  double kpCOM[3], kdCOM[3], kpBase[3], kdBase[3];
  double b_control[6];
  int counter = 0;
  // ros::spinOnce();
  _data->_legController->updateData();
  _data->_stateEstimator->run();
  // se_xfb[3] = 1.0; 
  //  int during = 10;
  // position & rpy desired
  for(int i = 0; i < 3; i++){
   
    v_des[i] = 0;
    omegaDes[i] = 0;
    rpy[i] = 0;
  }
    //rpy[2] = _data->_stateEstimator->getResult().rpy(2);
    //p_des[2] = 0.4; // standup height for AlienGo
    p_des[2] = 0.3; // standup height for A1
    _data->_stateEstimator->setContactPhase(contactphase);
  
    //std::cout << ros::Time::now() << std::endl;
    // rate.reset();
    _data->_legController->updateData();
    _data->_stateEstimator->run();
   
    if(motiontime > 5000){
     Vec3<double> v_des(0.03, 0.03, 0);
     double yaw_rate = 0;

    //  if(motiontime > 6000){
    //    v_des[0] = 0.1;
    //  }

    //  if(motiontime > 8000){
    //    v_des[0] = 0;
    //  }

    //  if(motiontime > 9000){
    //    v_des[0] = -0.1;
    //  }

    //  if(motiontime > 10000){
    //    v_des[0] = 0.05;
    //  }
     _data->_desiredStateCommand->setStateCommands(v_des, yaw_rate);

     QPloco.run(*_data);
     runQP = false;
     //rpy[0] = -0.5;

     pos << _data->_stateEstimator->getResult().vWorld(0) << " " << _data->_stateEstimator->getResult().vWorld(1) << 
    " " <<  _data->_stateEstimator->getResult().vWorld(2) << " " <<_data->_stateEstimator->getResult().position(0) << " " << _data->_stateEstimator->getResult().position(1) << " "
    << _data->_stateEstimator->getResult().position(2) << " " << _data->_stateEstimator->getResult().rpy(0) << " " << _data->_stateEstimator->getResult().rpy(1) << " " 
    << _data->_stateEstimator->getResult().rpy(2) << " " << _data->_legController->commands[0].pDes[0] << " " <<  _data->_legController->data[0].p[0] << " "
    <<  _data->_legController->commands[0].pDes[1] << " " <<  _data->_legController->data[0].p[1] << std::endl;
      
    }

     
  
    if(runQP){
      for (int i = 0; i < 4; i++) {
        se_xfb[i] = _data->_stateEstimator->getResult().orientation(i);
      }
     
      for (int i = 0; i < 3; i++) {
        //rpy[i] = 0;
        //rpy[i] = _data->_stateEstimator->getResult().rpy(i);
        //p_des[i] = _data->_stateEstimator->getResult().position(i);
        p_act[i] = _data->_stateEstimator->getResult().position(i);
        
        v_act[i] = _data->_stateEstimator->getResult().vBody(i);
        // v_des[i] = 0;
        //v_des[2] = 0.005;

        se_xfb[4 + i] = _data->_stateEstimator->getResult().position(i);
        se_xfb[7 + i] = _data->_stateEstimator->getResult().omegaBody(i);
        se_xfb[10 + i] = _data->_stateEstimator->getResult().vBody(i);
        
    // Set the translational and orientation gains
        kpCOM[i] = 50;   //_data->controlParameters->kpCOM(i);
        kdCOM[i] = 10;     //_data->controlParameters->kdCOM(i);
        kpBase[i] = 100;     //_data->controlParameters->kpBase(i);
        kdBase[i] = 10; //  _data->controlParameters->kdBase(i);
        }
      kpCOM[2] = 65;
      kdCOM[2] = 10;
      kpBase[0] = 400;
      kdBase[0] = 2;
      kpBase[1] = 200;
      //kpCOM[1] = 70;
  
    //Vec3<double> pFeetVec;
    Vec3<double> pFeetVecCOM;

    // Get the foot locations relative to COM
      for (int leg = 0; leg < 4; leg++) {
        // computeLegJacobianAndPosition(&_data->_quadruped, _data->_legController->data[leg].q,
        //                           (Mat3<double>*)nullptr, &pFeetVec, leg);
        //pFeetVecCOM = _data->_stateEstimator->getResult().rBody.transpose() *
                  //(_data->_quadruped->getHipLocation(leg) + pFeetVec);

        pFeetVecCOM =  _data->_stateEstimator->getResult().rBody.transpose() *
        (_data->_quadruped->getHipLocation(leg) + _data->_legController->data[leg].p);


        pFeet[leg * 3] = pFeetVecCOM[0];
        pFeet[leg * 3 + 1] = pFeetVecCOM[1];
        pFeet[leg * 3 + 2] = pFeetVecCOM[2];
        //std::cout << "pFeet" << leg << std::endl;
      }
      p_des[0] = p_act[0] + (pFeet[0] + pFeet[3] + pFeet[6] + pFeet[9])/4.0;
      p_des[1] = p_act[1] + (pFeet[1] + pFeet[4] + pFeet[7] + pFeet[10])/4.0;
    //  myfile << "\n";
     // std::cout << j << std::endl;
     //std::cout << "run QP" << std::endl;
    balanceController.set_alpha_control(0.01);
    balanceController.set_friction(0.6);
    balanceController.set_mass(_data->_quadruped->mass);
    balanceController.set_wrench_weights(COM_weights_stance, Base_weights_stance);
    balanceController.set_PDgains(kpCOM, kdCOM, kpBase, kdBase);
    balanceController.set_desiredTrajectoryData(rpy, p_des, omegaDes, v_des);
    balanceController.SetContactData(contactStateScheduled, minForces, maxForces);
    balanceController.updateProblemData(se_xfb, pFeet, p_des, p_act, v_des, v_act,
                                      O_err, _data->_stateEstimator->getResult().rpy(2));
   // balanceController.print_QPData();
    double fOpt[12];
    balanceController.solveQP_nonThreaded(fOpt);

    for (int leg = 0; leg < 4; leg++) {
        footFeedForwardForces.col(leg) << fOpt[leg * 3], fOpt[leg * 3 + 1],
        fOpt[leg * 3 + 2]; // force in world frame, need to convert to body frame

        _data->_legController->commands[leg].feedforwardForce = footFeedForwardForces.col(leg);
        
      }
     

    }

    _data->_legController->updateCommand();

  
 control.PowerProtect(lowCmd, lowState, 10);
}

void FSM_State::PDstand(){

  int counter = 0;
  
  // initial foot possition
  Mat34<double> init_foot_pos;
  _data->_legController->updateData();
  for(int i = 0; i < 4; i++){
    init_foot_pos.col(i) = _data->_legController->data[i].p;
  }
  double h = 0.4; // standup height
  double side_sign[4] = {-1, 1, -1, 1};
  Vec3<double> ff_force_world(0.5, 0, 0);

 
     double progress = counter * 0.001;
     if(progress > 1.)  {progress = 1.;}

    _data->_legController->updateData();  
    _data->_stateEstimator->run();    

      for(int i = 0; i < 4; i++){
        _data->_legController->commands[i].kpCartesian = Vec3<double>(400,400,900).asDiagonal();
        _data->_legController->commands[i].kdCartesian = Vec3<double>(20,20,20).asDiagonal();

        _data->_legController->commands[i].pDes << 0, side_sign[i] * 0.083, 0;
        
        _data->_legController->commands[i].pDes[2] = -h*progress + (1. - progress) * init_foot_pos(2, i);
       // _data->_legController->commands[i].feedforwardForce = _data->_stateEstimator->getResult().rBody.transpose() * ff_force_world;
       // std::cout << "leg  " << i << "  " << _data->_legController->data[i].p << std::endl;
      }

      _data->_legController->updateCommand();
      motiontime++;

}    