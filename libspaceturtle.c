#include <math.h>
#include <stdio.h>
#include "libspaceturtle.h"

int step_forward(struct body * root, struct world_config config, unsigned long long int ticks){
  struct body * current_body1;
  struct body * current_body2;
  double x_accel, y_accel, accel, x_dist, y_dist, distance;
  for(unsigned long long int i = 0; i <= ticks; i++){
    current_body1 = root;
    while(current_body1->next != NULL){
      x_accel = 0;
      y_accel = 0;
      current_body2 = root;
      while(current_body2->next){
	if((current_body1->next == current_body2->next) && current_body1->next)
	  current_body2 = current_body2->next;
	x_dist = current_body1->xpos - current_body2->xpos;
	y_dist = current_body1->ypos - current_body2->ypos;
	distance = sqrt(powl(x_dist, 2) + powl(y_dist, 2));
	accel = CALCACCEL(distance, current_body2->mass);
	x_accel += sinl(atanl(x_dist / y_dist)) * accel;
	y_accel += cosl(atanl(x_dist / y_dist)) * accel;
      }
      current_body1->xvel += x_accel / config.tickspersec;
      current_body1->yvel += y_accel / config.tickspersec;
      current_body1->xpos += current_body1->xvel / config.tickspersec;
      current_body1->ypos += current_body1->yvel / config.tickspersec;
      current_body1 = current_body1->next;
    }
  }
  return 0;
}
