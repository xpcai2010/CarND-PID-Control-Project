#ifndef PID_H
#define PID_H
#include <vector>
#include <uWS/uWS.h>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double cte_prev;

  // double min_error;
  // double max_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

/*
* twiddle variable
*/
 std::vector<double> dp;
 bool b_twiddle_init, b_twiddle_reset, b_increase, b_decrease;
 double best_error, total_error, twiddle_tolerance;
 int i_eval_steps, pid_idx; 

 double PID_hyper_prev;

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

  void Twiddle(double total_error);

  void Twiddle_Restart(uWS::WebSocket<uWS::SERVER> ws);
  
  void PID_Hyperparameter(int index, double increment);
  double return_Hyperparameter(int index);
};

#endif /* PID_H */
