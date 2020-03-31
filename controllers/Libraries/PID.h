// ------------------------------------------------------------------------------
// ! \file PID.h
// ! defines the PID controller
// ! based on https://github.com/geekfactory/PID
//
// ------------------------------------------------------------------------------

#ifndef PID_H
#define PID_H

#include <stdio.h>

struct pid_controller
{
  // Input, output and setpoint
  float *   input;     // !< Current process value
  float *   output;    // !< Corrective output
  float *   target;    // !< Controller target
  // Tuning parameters
  float     K;         // !< Stores the gain for the Overall term
  float     Ti;        // !< Stores the gain for the Integral term
  float     Td;        // !< Stores the gain for the Derivative term
  // Output minimum and maximum values
  float     outMin;    // !< Maximum value allowed at the output
  float     outMax;    // !< Minimum value allowed at the output
  // Variables for PID algorithm
  float     pterm;     // !< proportional term
  float     iterm;     // !< Integral term
  float     dterm;     // !< Differential term
  // Time related
  long long lasttime;  // !< Stores the time when the control loop ran last time
  float     deltaTime; // !< keep track of the deltas between calls
  // Operation mode
  int       automode;  // !< Defines if the PID controller is enabled or disabled
};

typedef struct pid_controller * pid_controller_t; // !< structure holding all the infos for a PID controller

/**
 *  Creates a new PID controller
 *
 *  \param pid Pointer to a pid_controller struct
 *  \param in Pointer to a float value for the process input
 *  \param out Pointer to put the controller output value
 *  \param set Pointer to the target value
 *  \param kp proportional gain
 *  \param ki integral gain
 *  \param kd derivative gain
 *
 *  \return a pid_controller_t controller handle
 **/
pid_controller_t pid_create(pid_controller_t pid, float* in, float* out, float* set, float K, float Ti, float Td);

/**
 *  Compute the output of the PID controller
 *
 *  \param pid PID controller instance
 **/
void pid_compute(pid_controller_t pid);

/**
 * Set new PID tuning parameters
 *
 *
 * \param pid The PID controller instance to modify
 * \param kp Proportional gain
 * \param ki Integral gain
 * \param kd Derivative gain
 */
void pid_controller_tune(pid_controller_t pid, float K, float Ti, float Td);

/**
 * Set the limits for the PID controller output
 *
 * \param pid The PID controller instance to modify
 * \param min The minimum output value for the PID controller
 * \param max The maximum output value for the PID controller
 */
void pid_limits(pid_controller_t pid, float min, float max);

/**
 * Enable automatic control using PID
 *
 * Enable the PID control loop. If manual output adjustment is needed you can
 * disable the PID control loop using pid_manual(). This function enables PID
 * automatic control at program start or after calling pid_manual()
 *
 * \param pid The PID controller instance to enable
 */
void pid_auto(pid_controller_t pid);

/**
 * Disable automatic process control
 *
 * Disable the PID control loop. User can modify the value of the output
 * variable and the controller will not overwrite it.
 *
 * \param pid The PID controller instance to disable
 */
void pid_manual(pid_controller_t pid);

/**
 *  Print PID info to console
 *
 *  \param pid the PID controller instance
 *
 **/
void pid_print(pid_controller_t pid);

/**
 *  prit the header for a CSV file
 *
 *  \param f pointer to a file
 **/
void filePrintHeader(FILE* f);

/**
 *  print data to a CSV file
 *
 *  \param f pointer to a file
 *  \param pid PID controller instance
 **/
void filePrintData(FILE* f, pid_controller_t pid);

#endif /* ifndef PID_H */