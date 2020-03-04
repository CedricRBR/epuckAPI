#ifndef _S01_FORWARD_BACKWARD_H_
#define _S01_FORWARD_BACKWARD_H_

#define SIMULATION 0

#if SIMULATION
#include "../API/webots/webotsAPI.h"
#else
#include "../API/epuck/epuckAPI.h"
#endif

/**
 *  Setup the robot, called once in the beginning of main
 *
 *  \return void
 **/
void robot_setup();

/**
 *  The robot main loop, called after robot_setup()
 *
 *  \return void
 **/
void robot_loop();

/**
 *  program main function
 *
 *  \param argc command line arguments
 *  \param argv pointer additional arguments
 *  \return 1 on success
 **/
int  main(int argc, char **argv);
#endif /* ifndef _S01_FORWARD_BACKWARD_H_ */