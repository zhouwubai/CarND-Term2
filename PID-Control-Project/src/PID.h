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
  * variable for twiddle
  */
  
  bool tunning_finished_;
  
  std::vector<double> d_coeffs_;
    
  double delta_tol_;
  
  int coeffs_idx_;
  
  int twiddle_state_;
  
  double best_error_;
  
  
  /*
  * variable for run
  */
  bool run_finished_;
  
  double run_err_max_;
  
  int run_n_min_;
  
  int run_n_max_;
  
  double run_error_;
  
  double t_weight_;
  
  int run_steps_;
  
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
  void Init(std::vector<double> coeffs);

  /*
  * Initialize twiddle setting
  */
  void InitTwiddle(bool finished, double delta_tol, double err_max, int n_min, int n_max, std::vector<double> d_coeffs);

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
  void ResetRun();
  
  void Run(double cte);
  
  double RunError();
  
  void Twiddle();
  
};

#endif /* PID_H */
