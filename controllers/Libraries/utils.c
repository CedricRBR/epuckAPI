#include "utils.h"

#include "../API/epuck/epuckAPI.h"
#include <math.h>

float toSteps(float mm)
{
  return (mm / WHEEL_CIRCUMFERENCE) * REDUCTION * STEPS_PER_REVOLUTION;
}

float toMM(int steps)
{
  return (steps * 1.0) / STEPS_PER_REVOLUTION / REDUCTION * WHEEL_CIRCUMFERENCE;
}

void turn ( float radius, int dir, float speed)
{
  int spd = speed * (radius - DIST_CENTER_WHEEL) / ( DIST_CENTER_WHEEL + radius);

  if(dir)
  {
    set_speed(speed, spd);
  }
  else
  {
    set_speed(spd, speed);
  }

  return; // DEBUG

  if(radius == 0)
  {
    if(dir)
    {
      set_speed(speed, -speed);
    }
    else
    {
      set_speed(-speed, speed);
    }
  }
  else if(radius<=DIST_CENTER_WHEEL)
  {
    float fwd = 1;
    float rev = (1.0 - radius / DIST_CENTER_WHEEL);

    if(dir)
    {
      set_speed(speed * fwd, speed * rev);
    }
    else
    {
      set_speed(speed * rev, speed * fwd);
    }
  }
  else
  {
    float big   = 1;
    float small = 1 - WHEEL_DIST / radius;

    if(dir)
    {
      set_speed(speed * big, speed * small);
    }
    else
    {
      set_speed(speed * small, speed * big);
    }
  }
} /* turn */

float distanceTravelled(int stepsLeft, int stepsRight)
{
  return (toMM(stepsLeft) + toMM(stepsRight)) * 0.5;
}

float radiusToDist(float radius)
{
  return 2 * PI * radius;
}

char* format_time(char *output)
{
  time_t      rawtime;
  struct tm * timeinfo;

  time ( &rawtime );
  timeinfo = localtime ( &rawtime );

  sprintf(output, "%d %d %d %d:%d:%d", timeinfo->tm_mday, timeinfo->tm_mon + 1, timeinfo->tm_year + 1900, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
  return output;
}

void printData(FILE* f)
{
  fprintf(f, "%hi, %hi, %hi,",      *(short int *)(&sensor[0]), *(short int *)(&sensor[2]), *(short int *)(&sensor[4]));
  fprintf(f, "%f,",                 *(float *)(&sensor[6]));
  fprintf(f, "%f,",                 *(float *)(&sensor[10]));
  fprintf(f, "%f,",                 *(float *)(&sensor[14]));
  fprintf(f, "%hi, %hi, %hi,",      *(short *)(&sensor[18]),     *(short *)(&sensor[20]),     *(short *)(&sensor[22]));
  fprintf(f, "%f,%f,%f,",           *(float *)(&sensor[24]),     *(float *)(&sensor[28]),     *(float *)(&sensor[32]));
  fprintf(f, "%hhu,",               *(unsigned char *)(&sensor[36]));
  fprintf(f, "%hi,%hi,%hi,%hi,",    *(short int *)(&sensor[37]), *(short int *)(&sensor[39]), *(short int *)(&sensor[41]), *(short int *)(&sensor[43]));
  fprintf(f, "%hi,%hi,%hi,%hi,",    *(short int *)(&sensor[45]), *(short int *)(&sensor[47]), *(short int *)(&sensor[49]), *(short int *)(&sensor[51]));
  fprintf(f, "%hi,%hi,%hi,%hi,",    *(short int *)(&sensor[53]), *(short int *)(&sensor[55]), *(short int *)(&sensor[57]), *(short int *)(&sensor[59]));
  fprintf(f, "%hi,%hi,%hi,%hi,",    *(short int *)(&sensor[61]), *(short int *)(&sensor[63]), *(short int *)(&sensor[65]), *(short int *)(&sensor[67]));
  fprintf(f, "%i,",                 *(short int *)(&sensor[69]));
  fprintf(f, "%hi, %hi, %hi, %hi,", *(short int *)(&sensor[71]), *(short int *)(&sensor[73]), *(short int *)(&sensor[75]), *(short int *)(&sensor[77]));
  fprintf(f, "%hi,",                *(short int *)(&sensor[79]));
  fprintf(f, "%hi,",                *(short int *)(&sensor[81]));
  fprintf(f, "%hi,%hi,%hi,",        *(short int *)(&sensor[90]), *(short int *)(&sensor[92]), *(short int *)(&sensor[94]));
  fprintf(f, "%s",                  format_time(t));
  fprintf(f, "\n");

} /* printData */

void printHeader(FILE* f)
{
  fprintf(f, "accX,accY,accZ,");
  fprintf(f, "accel,");
  fprintf(f, "orientation,");
  fprintf(f, "inclination,");
  fprintf(f, "gyroP,gyroR,gyroY,");
  fprintf(f, "magX,magY,magZ,");
  fprintf(f, "temp,");
  int i;

  for(i = 0; i<PROX_SENSORS_COUNT; ++i)
  {
    fprintf(f, "prox%i,", i);
  }

  for(i = 0; i<PROX_SENSORS_COUNT; ++i)
  {
    fprintf(f, "proxAmbient%i,", i);
  }

  fprintf(f, "ToF,");

  for(i = 0; i<MICROPHONE_COUNT; ++i)
  {
    fprintf(f, "mic%i,", i);
  }

  fprintf(f, "stepsLeft,");
  fprintf(f, "stepsRight,");

  for(i = 0; i<GROUND_SENSORS_COUNT; ++i)
  {
    fprintf(f, "gndAmbient%i,", i);
  }

  fprintf(f, "time");

  fprintf(f, "\n");
} /* printHeader */