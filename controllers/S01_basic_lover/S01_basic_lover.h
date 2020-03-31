// ------------------------------------------------------------------------------
// ! \file S01_basic_lover.h
// ! declares the basic lover functions
//
// ------------------------------------------------------------------------------
#ifndef _S01_BASIC_LOVER_H_
#define _S01_BASIC_LOVER_H_

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

#endif /* ifndef _S01_BASIC_LOVER_H_ */