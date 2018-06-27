#include "PID.h"
#include "math.h"
#include <iostream>

/*
* Initialize PID control, update the cross-track error, update PID
* coefficients, Twiddle, and calculate the total error (steering angle).
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double throttle_a, double throttle_b) {
  /*
  * Initialize PID controller with the values for proportional, 
  * integral and differential coefficients
  * This coefficients do not change with time during the simulation
  */
  
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
 
  this->throttle_a = throttle_a;
  this->throttle_b = throttle_b;

  this->p_error = 0; 
  this->i_error = 0; 
  this->d_error = 0; 
  // this->steer_value_prev = 0; 

  std::cout << "Init: Kp = " << this->Kp << std::endl;
  std::cout << "Init: Ki = " << this->Ki << std::endl;
  std::cout << "Init: Kd = " << this->Kd << std::endl;
  std::cout << "Init: throt_a = " << this->throttle_a << std::endl;
  std::cout << "Init: throt_b = " << this->throttle_b << std::endl;
}

void PID::UpdateError(double cte) {
  /*
  * Updates differential, proportional and integral error values
  */
  
  // d_error is the difference between this cte value and the previous
  this->d_error = (cte - p_error);

  // p_error is simply equal to the cte error value
  this->p_error = cte;

  // i_error is the integral error, equal to the sum of all cte values 
  // up to the current point in time
  this->i_error += cte;
  
  return;
}

void PID::UpdateSteerValuePrev(double steer_value) { 

  /*
   * Update the value of stter_value_prev 
   */

  this->steer_value_prev.push_back(steer_value);

  int current_n = this->steer_value_prev.size();

  if (current_n > N_QUEUE) { 
    this->steer_value_prev.pop_front();
  } 

  current_n = this->steer_value_prev.size();

  std::cout << "Current deque size: " << current_n << std::endl;
  for (int i = 0; i < current_n; i++) { 
    std::cout << this->steer_value_prev[i] << ", ";
  }

  std::cout << std::endl;
}

double PID::TotalError() {
  
  /*
   * Calculates the total error by multiplying proportional, 
   * differential and integral errors with their respective coefficients
   */

  // Return the total error value
  // This is used as a steering value in the main program
  double total_error = 0;  
  total_error -= this->Kp * this->p_error;
  total_error -= this->Ki * this->i_error;
  total_error -= this->Kd * this->d_error;

  int current_n = this->steer_value_prev.size();

  double steer_value = total_error;

  // Add N previous steer values to evarage out for smoothness
  for (int i = 0; i < current_n; i++) { 
    steer_value += this->steer_value_prev[i];
  }

  steer_value /= current_n + 1;

  // Remember the current total_error value for later
  PID::UpdateSteerValuePrev(total_error);

  return steer_value;
}

double PID::CalculateThrottle(double steer_value) { 

  /*
   * Calculates throttle from the steer_value and the 
   * throttle coefficients set at the beginning 
   */

  double throttle = (1 - fabs(steer_value)) * this->throttle_a + this->throttle_b;
  return throttle;
}
