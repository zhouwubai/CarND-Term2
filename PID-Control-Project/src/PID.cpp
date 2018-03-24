#include "PID.h"

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

void PID::InitTwiddle(double delta_tol, double err_max, int n_max){
    
    tunning_finished_ = false;
    
    d_coeffs_ = std::vector<double>(3, 1);
    
    delta_tol_ = delta_tol;
    
    run_finished_ = false;
    
    run_err_max_ = err_max;
    
    run_avg_error_ = 0.0;
    
    run_n_max_ = n_max;

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

double PID::Run(double cte){

}

double PID::Twiddle(double cte){
    
    
}
