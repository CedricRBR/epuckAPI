#include "empty_controller.h"

void robot_setup()
{
  init_robot();
}

void robot_loop()
{

  while (robot_go_on())
  {}
  cleanup_robot();
}

int main (int argc, char **argv)
{
    #if SIMULATION
    #else
  ip = argv[1];
    #endif

  robot_setup();
  robot_loop();
}