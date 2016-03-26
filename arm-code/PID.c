#include <PID.h>


void setPIDConstants(pid_data_t *pid, const float proportional_gain,
                     const float integral_gain, const float derivative_gain,
                     const float integral_guard) {
  pid->proportional_gain = proportional_gain;
  pid->integral_gain = integral_gain;
  pid->derivative_gain = derivative_gain;
  pid->integral_guard = integral_guard;
  pid->previous_error = 0;
  pid->integral_error = 0;
}

void updatePID(pid_data_t *pid, const float *current_error, const float *delta_t) {
  float error_differential = 0;

  pid->integral_error += *current_error * (*delta_t);
  pid->integral_error = constrain(pid->integral_error, -pid->integral_guard,
      pid->integral_guard);
  error_differential = (*current_error - pid->previous_error)/(*delta_t);

  pid->pid_output = (pid->proportional_gain * (*current_error)) +
                    (pid->integral_gain    * pid->integral_error) +
                    (pid->derivative_gain  * error_differential);

  pid->previous_error = *current_error;
} //end pidControl()

//fixed update PID is meant to be called at constant time intervals,
//therefore it does not need delta_t
void fixedUpdatePID(pid_data_t *pid, const float *current_error) {
  float error_differential = 0;

  pid->integral_error += *current_error;
  pid->integral_error = constrain(pid->integral_error, -pid->integral_guard,
      pid->integral_guard);

  error_differential = (*current_error - pid->previous_error);

  pid->pid_output = (pid->proportional_gain * (*current_error)) +
                    (pid->integral_gain    * pid->integral_error) +
                    (pid->derivative_gain  * error_differential);

  pid->previous_error = *current_error;
} //end pidControl()
