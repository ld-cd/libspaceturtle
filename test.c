#include <stdio.h>
#include <math.h>

#include "libspaceturtle.h"

int main(){
  struct body * earth, * cat;
  struct world_config ecsystem; //earthcatsystem
  earth = add_body(0, 0, 0, 0, 5.972 * powl(10, 24), NULL, NULL, NULL);
  cat = add_body(6771000, 0, 0, 7667, 10, NULL, earth, earth);
  ecsystem.tickspersec = 1000;
  for(int i = 0; i <= 100; i++){
    step_forward(earth, ecsystem, 60000);
    printf("minute: %5d, xpos: %12Lf, ypos: %12Lf, xvel: %12Lf, yvel: %12Lf\n", i + 1, cat->xpos / 1000, cat->ypos / 1000, cat->xvel, cat->yvel);
  }
  return 0;
}
