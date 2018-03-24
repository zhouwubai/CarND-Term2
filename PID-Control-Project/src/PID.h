#ifndef PID_H
#define PID_H

#include <iostream>
#include <vector>

class PID {
public:
  /*
  * Errors
  */
  std::vector<double> errors_;

  /*
  * Coefficients
  */
  std::vector<double> coeffs_;
  
  /*
  * variable for auto tunning
  */
  
  bool tunning_finished_;
  
  std::vector<double> d_coeffs_;
    
  double delta_tol_;
  
  bool run_finished_;
  
  double run_err_max_;
  
  double run_avg_error_;
  
  int run_n_max_;
  
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
  * Initialize twiddle setting
  */
  void InitTwiddle(double delta_tol, double err_max, int n_max);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  
  /*
  * auto tunning the parameters
  */
  double Run(double cte);
  
  double Twiddle(double cte);
  
};

#endif /* PID_H */
