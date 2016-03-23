#ifndef PID_H_
#define PID_H_

//Use this data structure to create a customized PID per motor
struct pid_data {
  float proportional_gain;
  float integral_gain;
  float derivative_gain;
  float previous_error;
  float integral_error;
  float integral_guard;
  float pid_output;
};

void setPIDConstants(pid_data *pid, const float proportional_gain,
                     const float integral_gain, const float derivative_gain,
                     const float integral_guard);

void updatePID(pid_data *pid, const float *current_error, const float *delta_t);

void fixedUpdatePID(pid_data *pid, const float *current_error);

#endif
