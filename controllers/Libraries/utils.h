// ------------------------------------------------------------------------------
// ! \file utils.h
// ! declares multiple utility functions
//
// ------------------------------------------------------------------------------

#ifndef _UTILS_H_
#define _UTILS_H_

#include <stdio.h>

#define STEPS_PER_REVOLUTION 20.0          // !< steps per motor revolution
#define REDUCTION            50.0          // !< gear reduction
#define MAX_STEPS_PER_SECOND 1200.0        // !< max speed in steps per second
#define WHEEL_DIAM           41.0          // !< wheel diameter
#define WHEEL_DIST           53.0          // !< distance between wheels
#define WHEEL_CIRCUMFERENCE  128.805298797 // !< wheel circumference, calculated from wheel diameter multiplied by pi
#define DIST_CENTER_WHEEL    26.5          // !< distance from center of robot to wheel
#define PI                   3.141592654   // !< PI redefine

static char t[8];
/**
 *  convert a mm into steps
 *
 *  \param mmPerSec the speed in mm/s
 *  \return the speed in steps/s
 **/
float toSteps(float mm);

/**
 *  convert steps into mm
 *
 *  \param steps the steps to convert
 **/
float toMM(int steps);

/**
 *  go in a circle of given radius, given direction and given speed
 *
 *  a radius of 0 will make the robot turn on the spot, a radius of 26.5 will make the robot turn with only one wheel
 *
 *  \param radius the turning radius (measured from the center of the robot)
 *  \param direction 0 for left, 1 for right
 *  \param speed the velocity of the outer wheel of the robot
 *  \return void
 **/
void turn ( float radius, int dir, float speed);

/**
 *  calculates the distance travelled by the robot.
 *
 *  param / return value
 **/
float distanceTravelled(int stepsLeft, int stepsRight);

/**
 *  return the distance to travel once around a circle of a given radius
 *
 *  \param radius the radius of the circle
 *  \return the circumference of the circle
 **/
float radiusToDist(float radius);

/**
 *  put a formatted time string into output
 *
 *  \param output the array holding the data
 *  \return pointer to the array holding the data
 **/
char* format_time(char *output);

/**
 *  print a dataset to a csv file
 *
 *  f the csv file
 **/
void printData(FILE* f);

/**
 *  print the header to a csv file
 *
 *  f pointer to the file
 **/
void printHeader(FILE* f);

#endif /* ifndef _UTILS_H_ */