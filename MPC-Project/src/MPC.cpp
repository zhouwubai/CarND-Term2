#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 8;
double dt = 0.15;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// weight for variables
const int w_cte = 500;
const int w_epsi = 200;
const int w_v = 1;

const int w_steer = 500;
const int w_throttle = 100;

const int w_steer_diff = 500;
const int w_throttle_diff = 100;

// Set the reference speed
double ref_v = 80;

// The solver takes all the state variables and actuator variables in a singular vector.
// Actuators only needs N - 1 value
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t steer_start = epsi_start + N;
size_t throttle_start = steer_start + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }
  
  // evalute the polynomial y given x
  AD<double> polyeval(const AD<double>& var_x){
    AD<double> var_y = 0.0;
    for(int i = 0; i < coeffs.size(); i ++){
        var_y += coeffs[i] * CppAD::pow(var_x, i); // int might cause problem
    }
    return var_y;
  }
  
  
  // calculate the derivative of polynomial at position x
  AD<double> derivative(const AD<double>& var_x){
    AD<double> y_prime = 0.0;
    // start from 1
    for(int i = 1; i < coeffs.size(); i++){
        y_prime += i * coeffs[i] * CppAD::pow(var_x, i-1);
    }
    return y_prime;
  }
  
  
  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    
    // first one is the cost
    fg[0] = 0;
    
    size_t t;
    
    // TODO: add more errors, distance to destination, smoothness etc
    // cost start from index 1, more errors can be added
    // The part of the cost based on the reference state.
    for (t = 0; t < N; t++) {
      fg[0] += w_cte * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += w_epsi * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += w_v * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (t = 0; t < N - 1; t++) {
      fg[0] += w_steer * CppAD::pow(vars[steer_start + t], 2);
      fg[0] += w_throttle * CppAD::pow(vars[throttle_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (t = 0; t < N - 2; t++) {
      fg[0] += w_steer_diff * CppAD::pow(vars[steer_start + t + 1] - vars[steer_start + t], 2);
      fg[0] += w_throttle_diff * CppAD::pow(vars[throttle_start + t + 1] - vars[throttle_start + t], 2);
    }
    
    // initial constraints, fg[0] is the cost
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];
    
    // set constraints
    for (t = 1; t < N; t ++){
        
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];
      
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];
      
      // actuators
      AD<double> steer0 = vars[steer_start + t -1];
      AD<double> throttle0 = vars[throttle_start + t - 1];
      
      // Here, we assume the psi0 is positive for turning left
      // that is why we need change the sign when passing to steer value
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * steer0 * dt / Lf);
      fg[1 + v_start + t] = v1 - (v0 + throttle0 * dt);
      
      // thereotically: cte1 = y0 + v0 * sin(epsi0) * dt - polyeval(x0)
      // a negative epsi0 means we should turn left
      fg[1 + cte_start + t] = cte1 - (polyeval(x0) - y0 + v0 * CppAD::sin(epsi0) * dt);
      fg[1 + epsi_start + t] = epsi1 - (psi0 - CppAD::atan(derivative(x0)) + v0 * steer0 * dt / Lf);
      
      //AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0;
      //AD<double> psi_des0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * x0 * x0);
      //fg[1 + cte_start + t] = cte1 - (f0 - y0 + v0 * CppAD::sin(epsi0) * dt);
      //fg[1 + epsi_start + t] = epsi1 - (psi0 - psi_des0 + v0 * steer0 * dt / Lf);
      
    }//END_OF_FOR
    
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;
  
  double x0 = state[0];
  double y0 = state[1];
  double psi0 = state[2];
  double v0 = state[3];
  double cte0 = state[4];
  double epsi0 = state[5];
  
  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = N * 6 + 2 * (N - 1);
  // TODO: Set the number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
  
  vars[x_start] = x0;
  vars[y_start] = y0;
  vars[psi_start] = psi0;
  vars[v_start] = v0;
  vars[cte_start] = cte0;
  vars[epsi_start] = epsi0;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  
  // TODO: Set lower and upper limits for variables.
  // x, y, psi, v, cte, epsi can be any number
  for(i = 0; i < steer_start; i ++){
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // put contraints for actuators, i.e., steer in radius and throttle
  for(i = steer_start; i < throttle_start; i ++){
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }
  
  // throttle constrain can be further adjust, larger
  for(i = throttle_start; i < n_vars; i ++){
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  
  // set the initial varaibles' lowerbound and upperbound
  // the must equal to, so lowerbound == upperbound == inital_value
  constraints_lowerbound[x_start] = x0;
  constraints_lowerbound[y_start] = y0;
  constraints_lowerbound[psi_start] = psi0;
  constraints_lowerbound[v_start] = v0;
  constraints_lowerbound[cte_start] = cte0;
  constraints_lowerbound[epsi_start] = epsi0;

  constraints_upperbound[x_start] = x0;
  constraints_upperbound[y_start] = y0;
  constraints_upperbound[psi_start] = psi0;
  constraints_upperbound[v_start] = v0;
  constraints_upperbound[cte_start] = cte0;
  constraints_upperbound[epsi_start] = epsi0;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  // options += "Numeric max_cpu_time         2.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  vector<double> results;
  int step = 2;
  
  results.push_back(solution.x[steer_start]);
  results.push_back(solution.x[throttle_start]);
  for(i = 0; i < N; i += step){
    results.push_back(solution.x[x_start + i]);
    results.push_back(solution.x[y_start + i]);
  }
  
  return results;
}
