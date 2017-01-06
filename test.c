#include <stdio.h>
#include <math.h>

#include "libspaceturtle.h"

int main(){
  struct body earth, cat;
  struct world_config ecsystem; //earthcatsystem
  cat.xpos = 0;
  cat.ypos = 0;
  cat.xvel = 0;
  cat.yvel = 0;
  cat.mass = 10;
  cat.next = &earth;
  earth.xpos = 6371000;
  earth.ypos = 0;
  earth.xvel = 0;
  earth.yvel = 0;
  earth.mass = 5.972 * powl(10, 24);
  earth.next = NULL;
  ecsystem.tickspersec = 1000;
  step_forward(&cat, ecsystem, 1000);
  printf("xpos: %Lf, xvel: %Lf\n", cat.xpos, cat.xvel);
  return 0;
}
