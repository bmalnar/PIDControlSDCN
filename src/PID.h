#ifndef PID_H
#define PID_H

#include<deque>

#define N_QUEUE 0

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
  double Kp;
  double Ki;
  double Kd;
  /*
  * Coefficients for calculating throttle
  * the equation is: throttle = (1 - steer_value) * a - b
  */
  double throttle_a;
  double throttle_b;

  /*
   * Store the previous steer value
   */
  std::deque<double> steer_value_prev;

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
  void Init(double Kp, double Ki, double Kd, double throttle_a, double throttle_b);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

 /*
  * Calculate throttle based on steer value 
  */
  double CalculateThrottle(double steer_value);

  /*
   * Update the value of stter_value_prev 
   */
  void UpdateSteerValuePrev(double steer_value);
};

#endif /* PID_H */
