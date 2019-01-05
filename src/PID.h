#ifndef PID_H
#define PID_H

#include <iostream>
#include <uWS/uWS.h>
#include <math.h>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp_;
  double Ki_;
  double Kd_;

  bool is_first ;

  std::vector<double> p;
  std::vector<double> dp;

  int param_index;
  double error;
  double best_error;
  int num_step;
  int num_echo;
  double echoes;

  bool try_positive;


  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /* 
  * Twiddle parameter Optimization
  * cte: cross track error
  * tune_index: if tune_index > 2 ,tune the kp,ki,kd parameters ,or tune the tune_index parameter
  */
  void Twiddle(double cte,int tune_index);

  /*
  * Update parameter 
  */
  void UpdateParam(double Kp,double Ki,double Kd);
};

#endif /* PID_H */
