#include <iostream>

#include <vector>
#include <math.h>
#include "Eigen-3.3/Eigen/Dense"

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;


vector<double> JMT(vector<double> &start, vector<double> &end, double T) {
  MatrixXd A = MatrixXd(3, 3);
  A << T*T*T, T*T*T*T, T*T*T*T*T,
       3*T*T, 4*T*T*T,5*T*T*T*T,
       6*T, 12*T*T, 20*T*T*T;

  MatrixXd B = MatrixXd(3,1);
  B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
       end[1]-(start[1]+start[2]*T),
       end[2]-start[2];

  MatrixXd Ai = A.inverse();

  MatrixXd C = Ai*B;

  vector <double> result = {start[0], start[1], .5*start[2]};

  for(int i = 0; i < C.size(); ++i) {
    result.push_back(C.data()[i]);
  }

  return result;
}


vector<vector<double>> getTrajectory(vector<double> start, double target_speed, double max_jerk, double max_acceleration){
  std::cout<<"getTraj called with: ("<<start[0]<<", "<<start[1]<<", "<<start[2]<<"), target_speed="<<target_speed<<std::endl;
  vector<vector<double>> trajectory;
  vector<double> next_start = {start[0], start[1], start[2]};
  /*
   *  this covers the case where the desired change in velocity is small relative to the initial acceleration and overshoot is inevitable
   *  - it brings acceleration to zero in minimum time & distance, with velocity overshooting the desired velocity
   *  - alternatively, this condition can also be met where the acceleration is the opposite sign of the velocity. There's no harm in continuing in this case, so we do so.
   */
  if (fabs(start[2]) >= (max_jerk/50.0) && fabs(target_speed - start[1]) <= 0.5 * pow(start[2], 2)/max_jerk){
    std::cout<<"traj: acceleration reduction"<<std::endl;
    double t = 0.02* floor(50*fabs(start[2])/max_jerk);
    double a = 0;
    double v = start[1] + start[2]*t + 0.5*((start[2]>0)? -1:1)*max_jerk*pow(t,2);
    double s = start[0] + start[1]*t + 0.5*start[2]*pow(t,2) + (1.0/6.0)*((start[2]>0)? -1.0:1.0)*max_jerk*pow(t,3);
    vector<double> end = {s, v, a};
    //end[0] = s; end[1]=v; end[2]=a;
    //end = {s, v, a};
    vector<double> jk = JMT(start, end, t);
    MatrixXd S = MatrixXd(3,1);
    S << jk[3], jk[4], jk[5];
    for (int i =1; i <= 50*t; i++){
      double T = 0.02*i;
      MatrixXd A = MatrixXd(3, 3);
      A << T*T*T, T*T*T*T, T*T*T*T*T,
      3*T*T, 4*T*T*T,5*T*T*T*T,
      6*T, 12*T*T, 20*T*T*T;
      MatrixXd C = A*S;
      next_start[0] =  C.data()[0] + start[0] + start[1]*T+0.5*start[2]*pow(T,2);
      next_start[1] = C.data()[1] + start[1] + T*start[2];
      next_start[2] = C.data()[2] + start[2];
      trajectory.push_back({next_start[0], next_start[1], next_start[2]});
    }
    for(int i=0; i<3; i++){
      start[i] = next_start[i];
    }
  }
  // time to bring acceleration to 0 - possibly negative
  double t_a = -0.02* floor(50*start[2] / (((target_speed > start[1])? 1.0:-1.0)*max_jerk));
  double v_star = start[1] + start[2]*t_a -0.5*max_jerk*t_a*fabs(t_a);
  // time to go from zero acceleration to peak/ minimum acceleration
  double t_r = 0.02* floor(50*std::min(sqrt(fabs(target_speed - v_star)/ max_jerk), max_acceleration/max_jerk));
  double a1 = start[2] + ((target_speed > start[1])? 1.0:-1.0)*max_jerk*(t_r+t_a);
  double v1 = start[1]+start[2]*(t_a+t_r) + 0.5*((target_speed > start[1])? 1.0:-1.0)*max_jerk*pow(t_a+t_r,2);
  double s1 = start[0] + start[1]*(t_a+t_r)+0.5*start[2]*pow(t_a+t_r,2)+(1.0/6.0)*((target_speed > start[1])? 1.0:-1.0)*max_jerk*pow(t_a+t_r,3);
  double a3 = a1 + ((target_speed > start[1])? -1.0:1.0)*max_jerk*(t_r);
  double v3 = v1 + a1*(t_r) + 0.5*((target_speed > start[1])? -1.0:1.0)*max_jerk*pow(t_r,2);
  double s3 = s1 + v1*(t_r)+0.5*a1*pow(t_r,2)+(1.0/6.0)*((target_speed > start[1])? -1.0: 1.0)*max_jerk*pow(t_r,3);
  // time at steady max/ min acceleration
  double t_s = 0.02* floor(50*(fabs(target_speed - v3))/ max_acceleration);
  if (t_r + t_a > 0){
    std::cout<<"traj: jerk to max"<<std::endl;
    vector<double> end = {s1, v1, a1};
    vector<double> jk = JMT(start, end, t_a+t_r);
    MatrixXd S = MatrixXd(3,1);
    S << jk[3], jk[4], jk[5];
    for (int i =1; i <= 50*(t_a+t_r); i++){
      double T = 0.02*i;
      MatrixXd A = MatrixXd(3, 3);
      A << T*T*T, T*T*T*T, T*T*T*T*T,
           3*T*T, 4*T*T*T,5*T*T*T*T,
	   6*T, 12*T*T, 20*T*T*T;
      MatrixXd C = A*S;
      next_start[0] =  C.data()[0] + start[0] + start[1]*T+0.5*start[2]*pow(T,2);
      next_start[1] = C.data()[1] + start[1] + T*start[2];
      next_start[2] = C.data()[2] + start[2];
      trajectory.push_back({next_start[0], next_start[1], next_start[2]});
    }
    for(int i=0; i<3; i++){
      start[i] = next_start[i];
    }
  }
  if (t_s > 0){
    std::cout<<"traj: peak acceleration"<<std::endl;
    for (int i =1; i <= 50*(t_s); i++){
      double T = 0.02*i;
      next_start[2] = start[2];
      next_start[1] = start[1]+ T*start[2];
      next_start[0] = start[0] + start[1]*T+0.5*start[2]*pow(T,2);
      trajectory.push_back({next_start[0], next_start[1], next_start[2]});
    }
    for(int i=0; i<3; i++){
      start[i] = next_start[i];
    }
  }
  if (t_r > 0){
    std::cout<<"traj: jerk down to max"<<std::endl;
    double a = start[2] + ((target_speed > start[1])? -1.0:1.0)*max_jerk*(t_r);
    double v = start[1]+start[2]*(t_r) + 0.5*((target_speed > start[1])? -1.0:1.0)*max_jerk*pow(t_r,2);
    double s = start[0] + start[1]*(t_r)+0.5*start[2]*pow(t_r,2)+(1.0/6.0)*((target_speed > start[1])? -1.0: 1.0)*max_jerk*pow(t_r,3);
    // vector<double> end = {s, v, a};
    vector<double> end = {s, target_speed, a};
    vector<double> jk = JMT(start, end, t_r);
    MatrixXd S = MatrixXd(3,1);
    S << jk[3], jk[4], jk[5];
    for (int i=1; i <= 50*(t_r); i++){
      double T = 0.02*i;
      MatrixXd A = MatrixXd(3, 3);
      A << T*T*T, T*T*T*T, T*T*T*T*T,
           3*T*T, 4*T*T*T,5*T*T*T*T,
           6*T, 12*T*T, 20*T*T*T;
      MatrixXd C = A*S;
      next_start[0] =  C.data()[0] + start[0] + start[1]*T+0.5*start[2]*pow(T,2);
      next_start[1] = C.data()[1] + start[1] + T*start[2];
      next_start[2] = C.data()[2] + start[2];
      trajectory.push_back({next_start[0], next_start[1], next_start[2]});
    }
    for(int i=0; i<3; i++){
      start[i] = next_start[i];
    }
  }
  if (trajectory.size()<32 && fabs(target_speed - start[1])<0.5 && fabs(start[2]) < (max_jerk/50.0)){
    std::cout<<"traj: cruising at steady speed"<<std::endl;
    for (int i=1; i<17; i++){
      next_start[0] = start[0] + start[1]*0.02*(double)i;
      next_start[1] = start[1];
      next_start[2] = 0.0;
      trajectory.push_back({next_start[0], next_start[1], next_start[2]});
    }
    for(int i=0; i<3; i++){
      start[i] = next_start[i];
    }
  }
  // finally, we could fill in extra points maintaining target speed if the trajectory vector is short?
  std::cout<<"trajectory ends with s,v,a ("<<trajectory[trajectory.size()-1][0]<<", "<<trajectory[trajectory.size()-1][1]<<", "<<trajectory[trajectory.size()-1][2]<<")"<<std::endl;
  return trajectory;
}

vector<vector<double>> getTailTrajectory(vector<double> start, vector<double> end, double time){
  vector<vector<double>> trajectory;
  vector<double> jk = JMT(start, end, time);
  MatrixXd S = MatrixXd(3,1);
  S << jk[3], jk[4], jk[5];
  for (int i=1; i <= 50*(time); i++){
    double T = 0.02*i;
    MatrixXd A = MatrixXd(3, 3);
    A << T*T*T, T*T*T*T, T*T*T*T*T,
         3*T*T, 4*T*T*T,5*T*T*T*T,
         6*T, 12*T*T, 20*T*T*T;
    MatrixXd C = A*S;
    vector<double> next_start = {0,0,0};
    next_start[0] =  C.data()[0] + start[0] + start[1]*T+0.5*start[2]*pow(T,2);
    next_start[1] = C.data()[1] + start[1] + T*start[2];
    next_start[2] = C.data()[2] + start[2];
    trajectory.push_back({next_start[0], next_start[1], next_start[2]});
  }
  return trajectory;
}

vector<vector<double>> getLaneShift(double start_s, double end_s, double max_jerk, double max_acceleration){
  vector<vector<double>> trajectory;
  double t_q = 0.02* floor(50*pow(0.5*(fabs(end_s-start_s))/max_jerk, (1.0/3.0)));
  vector<double> start = {start_s, 0, 0};
  // this probably won't work - I'll have to break it into quarters
  // quarter point
  vector<double> end = {(1.0/6.0)*max_jerk*pow(t_q,3.0), 0.5*max_jerk*pow(t_q,2.0), max_jerk*t_q};
  vector<double> jk = JMT(start, end, t_q);
  vector<double> next_start={0,0,0};
  MatrixXd S = MatrixXd(3,1);
  S << jk[3], jk[4], jk[5];
  for (int i=1; i <= 50*(t_q); i++){
    double T = 0.02*i;
    MatrixXd A = MatrixXd(3, 3);
    A << T*T*T, T*T*T*T, T*T*T*T*T,
         3*T*T, 4*T*T*T,5*T*T*T*T,
         6*T, 12*T*T, 20*T*T*T;
    MatrixXd C = A*S;
    next_start[0] =  C.data()[0] + start[0] + start[1]*T+0.5*start[2]*pow(T,2);
    next_start[1] = C.data()[1] + start[1] + T*start[2];
    next_start[2] = C.data()[2] + start[2];
    trajectory.push_back({next_start[0], next_start[1], next_start[2]});
  }
  for(int i=0; i<3; i++){
    start[i] = next_start[i];
  }
  end = {max_jerk*pow(t_q,3.0), max_jerk*pow(t_q,2.0), 0};
  jk = JMT(start, end, t_q);
  S = MatrixXd(3,1);
  S << jk[3], jk[4], jk[5];
  for (int i=1; i <= 50*(t_q); i++){
    double T = 0.02*i;
    MatrixXd A = MatrixXd(3, 3);
    A << T*T*T, T*T*T*T, T*T*T*T*T,
         3*T*T, 4*T*T*T,5*T*T*T*T,
         6*T, 12*T*T, 20*T*T*T;
    MatrixXd C = A*S;
    next_start[0] =  C.data()[0] + start[0] + start[1]*T+0.5*start[2]*pow(T,2);
    next_start[1] = C.data()[1] + start[1] + T*start[2];
    next_start[2] = C.data()[2] + start[2];
    trajectory.push_back({next_start[0], next_start[1], next_start[2]});
  }
  for(int i=0; i<3; i++){
    start[i] = next_start[i];
  }
  end = {(11.0/6.0)*max_jerk*pow(t_q,3.0), 0.5*max_jerk*pow(t_q,2.0), -max_jerk*t_q};
  jk = JMT(start, end, t_q);
  S = MatrixXd(3,1);
  S << jk[3], jk[4], jk[5];
  for (int i=1; i <= 50*(t_q); i++){
    double T = 0.02*i;
    MatrixXd A = MatrixXd(3, 3);
    A << T*T*T, T*T*T*T, T*T*T*T*T,
         3*T*T, 4*T*T*T,5*T*T*T*T,
         6*T, 12*T*T, 20*T*T*T;
    MatrixXd C = A*S;
    next_start[0] =  C.data()[0] + start[0] + start[1]*T+0.5*start[2]*pow(T,2);
    next_start[1] = C.data()[1] + start[1] + T*start[2];
    next_start[2] = C.data()[2] + start[2];
    trajectory.push_back({next_start[0], next_start[1], next_start[2]});
  }
  for(int i=0; i<3; i++){
    start[i] = next_start[i];
  }
  end = {2.0*max_jerk*pow(t_q,3.0), 0, 0};
  jk = JMT(start, end, t_q);
  S = MatrixXd(3,1);
  S << jk[3], jk[4], jk[5];
  for (int i=1; i <= 50*(t_q); i++){
    double T = 0.02*i;
    MatrixXd A = MatrixXd(3, 3);
    A << T*T*T, T*T*T*T, T*T*T*T*T,
         3*T*T, 4*T*T*T,5*T*T*T*T,
         6*T, 12*T*T, 20*T*T*T;
    MatrixXd C = A*S;
    next_start[0] =  C.data()[0] + start[0] + start[1]*T+0.5*start[2]*pow(T,2);
    next_start[1] = C.data()[1] + start[1] + T*start[2];
    next_start[2] = C.data()[2] + start[2];
    trajectory.push_back({next_start[0], next_start[1], next_start[2]});
  }
  for(int i=0; i<3; i++){
    start[i] = next_start[i];
  }
  return trajectory;
}
