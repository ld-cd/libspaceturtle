#include <stdio.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>

#include "libspaceturtle.h"

int main(){
  struct body * earth, * cat, * root;
  struct world_config ecsystem; //earthcatsystem
  unsigned int seed = rand();
  clock_t s, e;
  ecsystem.tickspersec = 1000;
  
  earth = add_body(0, 0, 0, 0, 5.972 * powl(10, 24), 0, NULL, NULL, NULL);
  cat = add_body(6771000, 0, 0, 7667, 10, 0, NULL, earth, earth);
  for(int i = 0; i <= 100; i++){
    step_forward(earth, ecsystem, 60000);
    printf("minute: %5d, xpos: %12Lf, ypos: %12Lf, xvel: %12Lf, yvel: %12Lf\n", i + 1, cat->xpos / 1000, cat->ypos / 1000, cat->xvel, cat->yvel);
  }
  delete_body(NULL, earth);
  
  earth = add_body(0, 0, 0, 0, 5.972 * powl(10, 24), 0, NULL, NULL, NULL);
  cat = add_body(6771000, 0, 0, 7667, 10, 0, NULL, earth, earth);
  for(int i = 0; i <= 100; i++){
    step_forward_tctd(earth, ecsystem, 60000);
    printf("minute: %5d, xpos: %12Lf, ypos: %12Lf, xvel: %12Lf, yvel: %12Lf\n", i + 1, cat->xpos / 1000, cat->ypos / 1000, cat->xvel, cat->yvel);
  }
  delete_body(NULL, earth);
  srand(seed);
  root = add_body(rand(), rand(), rand(), rand(), rand(), 0, NULL, NULL, NULL);
  for(int i = 0; i < 299; i++){
    add_body(rand(), rand(), rand(), rand(), rand(), 0, NULL, NULL, root);
  }
  s = clock();
  step_forward(root, ecsystem, 1000);
  e = clock();
  printf("%fs\n", (double) (e - s) / CLOCKS_PER_SEC);
  delete_body(NULL, root);
  
  srand(seed);
  root = add_body(rand(), rand(), rand(), rand(), rand(), 0, NULL, NULL, NULL);
  for(int i = 0; i < 299; i++){
    add_body(rand(), rand(), rand(), rand(), rand(), 0, NULL, NULL, root);
  }
  s = clock();
  step_forward_tctd(root, ecsystem, 1000);
  e = clock();
  printf("%fs\n", (double) (e - s) / CLOCKS_PER_SEC);
  delete_body(NULL, root);
  
  return 0;
}
