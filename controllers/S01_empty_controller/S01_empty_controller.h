// ------------------------------------------------------------------------------
// ! \file S01_empty_controller.h
// ! declares the empty controller functions
//
// ------------------------------------------------------------------------------

#ifndef _EMPTY_CONTROLLER_H_
#define _EMPTY_CONTROLLER_H_
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
int main(int argc, char **argv);
#endif /* ifndef _S01_EMPTY_CONTROLLER_H_ */