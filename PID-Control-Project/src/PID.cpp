#include "PID.h"
#include <math.h>

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(std::vector<double> coeffs) {
    
    coeffs_ = coeffs;
    errors_ = std::vector<double>(3, 0);
    tunning_finished_ = true;// default true
    
    std::cout << "Kp: " << coeffs_[0] << " Ki: " << coeffs_[1] << " Kd: " << coeffs_[2] << std::endl;
}

void PID::InitTwiddle(bool finished, double delta_tol, double err_max, int n_min, int n_max, std::vector<double> d_coeffs){
    
    tunning_finished_ = finished;
    d_coeffs_ = d_coeffs;
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
    
    // set errors_ to 0
    std::fill(errors_.begin(), errors_.end(), 0);
}

void PID::UpdateError(double cte) {
    errors_[2] = cte - errors_[0];
    errors_[0] = cte;
    errors_[1] += cte;
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


/*
* calculate average error for each run
* Note: this should consider running step, more the better
*/
double PID::RunError(){

    double avg_err = 0.0;
    // if it fail even before run_n_min_, set error max
    if (run_steps_ < run_n_min_){
        avg_err = std::numeric_limits<double>::max();
    } else {
        avg_err = run_error_ / (run_steps_ - run_n_min_);
    }
    
    // consider the steps time_lever in (0, 1]
    double time_lever = exp(1.0 -  run_n_max_ / (double) run_steps_ );
    
    return avg_err / time_lever;
}


void PID::Twiddle(){
    
    if (run_finished_ == false){
        std::cout << "Last run has not finished yet." << std::endl;
        return;
    }
    
    // calculate the sum of delta
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
        std::cout << "state: " << twiddle_state_ << " index: " << coeffs_idx_ << std::endl;
        std::cout << "Kp: " << coeffs_[0] << " Ki: " << coeffs_[1] << " Kd: " << coeffs_[2] << std::endl;
        std::cout << "dp: " << d_coeffs_[0] << " di: " << d_coeffs_[1] << " dd: " << d_coeffs_[2] << std::endl;
        std::cout << "best error: " << best_error_ << std::endl;
        
    } else {
        tunning_finished_ = true;
        // output best parameters
        std::cout << "Tunning finished" << std::endl;
        std::cout << "Kp: " << coeffs_[0] << " Ki: " << coeffs_[1] << " Kd: " << coeffs_[2] << std::endl;
    }
}
