#include "PID.h"
#include <math.h>

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    
    coeffs_ = std::vector<double>{Kp, Ki, Kd};
    errors_ = std::vector<double>(3, 0);
    
    std::cout << "Kp: " << coeffs_[0] << " Ki: " << coeffs_[1] << " Kd: " << coeffs_[2] << std::endl;
}

void PID::InitTwiddle(double delta_tol, double err_max, int n_min, int n_max){
    
    tunning_finished_ = false;
    d_coeffs_ = std::vector<double>(3, 1);
    coeffs_idx_ = 0;
    twiddle_state_ = 0;
    delta_tol_ = delta_tol;
    best_error_ = std::numeric_limits<double>::max();
    
    run_err_max_ = err_max;
    run_n_min_ = n_min;
    run_n_max_ = n_max;
    
    ResetRun();
}

void PID::ResetRun(){
    run_finished_ = false;
    run_error_ = 0.0;
    run_steps_ = 0;
}

void PID::UpdateError(double cte) {
    errors_[0] = cte;
    errors_[1] += cte;
    errors_[2] = cte - errors_[2];
}

double PID::TotalError() {
    double total_err = 0.0;
    for (int i = 0; i < coeffs_.size(); i++){
        total_err -= coeffs_[i] * errors_[i];
    }
    
    return total_err;
}


void PID::Run(double cte){
    
    // only consider steps after several warmup
    if (run_steps_ > run_n_min_){
        run_error_ += cte * cte;
    }
    run_steps_ ++;
    
    /*
    * Two parameter control each run, if it runs to run_n_max_ steps
    * if cte > run_err_max_, it means it is losing control so terminate run earlier
    */
    if (run_steps_ > run_n_max_ || fabs(cte) > run_err_max_){
        run_finished_ = true;
    }
}


double PID::RunError(){

    double avg_err = 0.0;
    if (run_steps_ < run_n_min_){
        avg_err = std::numeric_limits<double>::max();
    } else {
        avg_err = run_error_ / (run_steps_ - run_n_min_);
    }
    return avg_err;
}


void PID::Twiddle(){
    
    double d_sum = 0.0;
    for (int i = 0; i < d_coeffs_.size(); i++){
        d_sum += d_coeffs_[i];
    }
    
    double err = 0.0;
    // twiddle finished standard
    if (d_sum > delta_tol_){
    
        switch (twiddle_state_) {
            case 0:
                //initial twiddle, just udpate the best_error
                best_error_ = RunError();
                twiddle_state_ = 1;
                break;
                
            case 1:
                // add delta for new parameters
                coeffs_[coeffs_idx_] += d_coeffs_[coeffs_idx_];
                twiddle_state_ = 2;
                break;
                
            case 2:
                // added delta, now check the avg_error
                err = RunError();
                if (err < best_error_){
                    best_error_ = err;
                    d_coeffs_[coeffs_idx_] *= 1.1;
                    
                    // restart for next parameter
                    twiddle_state_ = 1;
                    coeffs_idx_ += 1;
                    coeffs_idx_ = coeffs_idx_ % 3;
                } else {
                    // add delta fail, try decrease
                    coeffs_[coeffs_idx_] -= 2 * d_coeffs_[coeffs_idx_];
                    twiddle_state_ = 3;
                }
                break;
            
            case 3:
                // decreased delta, now check the avg_error
                err = RunError();
                if (err < best_error_){
                    best_error_ = err;
                    d_coeffs_[coeffs_idx_] *= 1.1;
                } else {
                    // tried increase and decrease, all failed, try small delta
                    coeffs_[coeffs_idx_] += d_coeffs_[coeffs_idx_];
                    d_coeffs_[coeffs_idx_] *= 0.9;
                }
                
                // restart next parameter
                twiddle_state_ = 1;
                coeffs_idx_ += 1;
                coeffs_idx_ = coeffs_idx_ % 3;
                break;
                
            default:
                break;
        }//END_SWITCH
        
        //output best_error_
        std::cout << "best error: " << best_error_ << std::endl;
        
    } else {
        tunning_finished_ = true;
        // output best parameters
        std::cout << "Kp: " << coeffs_[0] << " Ki: " << coeffs_[1] << " Kd: " << coeffs_[2] << std::endl;
    }
}
