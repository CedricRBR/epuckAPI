// ------------------------------------------------------------------------------
// ! \file PID.c
// ! Declares the PID controller
//
// ------------------------------------------------------------------------------

#include "PID.h"

#include <stdint.h>

#if SIMULATION
#include "../API/webots/webotsAPI.h"
#else
#include "../API/epuck/epuckAPI.h"
#include <sys/time.h>
#endif

long long get_millis()
{
  struct timeval tv;

  gettimeofday(&tv, NULL);
  return (((long long)tv.tv_sec) * 1000) + (tv.tv_usec / 1000);
}

pid_controller_t pid_create(pid_controller_t pid, float* in, float* out, float* target, float K, float Ti, float Td)
{
  pid->input    = in;
  pid->output   = out;
  pid->target   = target;
  pid->automode = 0;
  pid_limits(pid, 0, 255);
  pid_controller_tune(pid, K, Ti, Td);

  #if SIMULATION
  pid->lasttime = 0;
  #else
  pid->lasttime = get_millis();
  #endif

  return pid;
} /* pid_create */

void pid_compute(pid_controller_t pid)
{
  // Check if control is enabled
  if (!pid->automode)
  {
    return;
  }

  float in       = *(pid->input);
  float previous = pid->pterm;

  // get deltatime
  #if SIMULATION
  pid->deltaTime = TIME_STEP;
  #else
  pid->deltaTime = get_millis() - pid->lasttime;
  #endif

  // pterm
  pid->pterm  = (*(pid->target)) - in;

  // iterm
  pid->iterm += pid->pterm * (1000 / pid->deltaTime);

  // dterm
  pid->dterm  = (pid->pterm - previous) * (pid->deltaTime / 1000);

  // Compute PID output
  float out = pid->K * (
   pid->pterm +
   1.0 / pid->Ti * pid->iterm +
   pid->Td * pid->dterm
   );

  // Apply limit to output value
  if(out > pid->outMax)
  {
    pid->iterm -= (out - pid->outMax);
    out         = pid->outMax;
  }
  else if(out < pid->outMin)
  {
    pid->iterm += (pid->outMin - out);
    out         = pid->outMin;
  }

  // Output to pointed variable
  (*pid->output) = out;

  // update lasttime
  pid->lasttime += pid->deltaTime;

} /* pid_compute */

void pid_controller_tune(pid_controller_t pid, float K, float Ti, float Td)
{
  // Check for validity
  if ((K < 0) || (Ti < 0) || (Td < 0))
  {
    return;
  }

  pid->K  = K;
  pid->Ti = Ti;
  pid->Td = Td;

} /* pid_controller_tune */

void pid_limits(pid_controller_t pid, float min, float max)
{
  if (min >= max) {return;}

  pid->outMin = min;
  pid->outMax = max;

  // Adjust output to new limits
  if (pid->automode)
  {
    if (*(pid->output) > pid->outMax)
    {
      *(pid->output) = pid->outMax;
    }
    else if (*(pid->output) < pid->outMin)
    {
      *(pid->output) = pid->outMin;
    }

    if (pid->iterm > pid->outMax)
    {
      pid->iterm = pid->outMax;
    }
    else if (pid->iterm < pid->outMin)
    {
      pid->iterm = pid->outMin;
    }
  }
} /* pid_limits */

void pid_auto(pid_controller_t pid)
{
  // If going from manual to auto
  if (!pid->automode)
  {
    pid->iterm = *(pid->output);

    if (pid->iterm > pid->outMax)
    {
      pid->iterm = pid->outMax;
    }
    else if (pid->iterm < pid->outMin)
    {
      pid->iterm = pid->outMin;
    }

    pid->automode = 1;
  }
} /* pid_auto */

void pid_manual(pid_controller_t pid)
{
  pid->automode = 0;
}

void pid_print(pid_controller_t pid)
{
  printf("-------------------\n");
  printf("in:\t%f\n",     *(pid->input));
  printf("out:\t%f\n",    *(pid->output));
  printf("target:\t%f\n", *(pid->target));
  printf("pterm:\t%f\n",  pid->pterm);
  printf("iterm:\t%f\n",  pid->iterm);
  printf("dterm:\t%f\n",  pid->dterm);
  printf("deltaT:\t%f\n", pid->deltaTime);
  printf("\n");
}

void filePrintHeader(FILE* f)
{
  fprintf(f, "in,out,target,pterm,iterm,dterm,Kp,Ti,Td,deltaTime,lasttime,outMin,outMax\n");
}

void filePrintData(FILE* f, pid_controller_t pid)
{
  fprintf(f, "%f,%f,%f,",   *(pid->input), *(pid->output), *(pid->target));
  fprintf(f, "%f,%f,%f,", pid->pterm,      pid->iterm,     pid->dterm);
  fprintf(f, "%f,%f,%f,", pid->K,          pid->Ti,        pid->Td);
  fprintf(f, "%f,%lli,",  pid->deltaTime,  pid->lasttime);
  fprintf(f, "%f,%f\n",   pid->outMin,     pid->outMax);
}