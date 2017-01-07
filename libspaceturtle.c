#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "libspaceturtle.h"

int step_forward(struct body * root, struct world_config config, unsigned long long int ticks){
  struct body * current_body1;
  struct body * current_body2;
  double x_accel, y_accel, accel, x_dist, y_dist, distance;
  if(!root)
    return -1;
  for(unsigned long long int i = 0; i < ticks; i++){
    current_body1 = root;
    while(current_body1 != NULL){
      x_accel = 0;
      y_accel = 0;
      current_body2 = root;
      while(current_body2 != NULL){
	while((current_body2->next == current_body1->next) || (current_body2->mass == 0.0)){
	  current_body2 = current_body2->next;
	  if(current_body2 == NULL)
	    break;
	}
	if(current_body2 == NULL)
	  break;
	x_dist = current_body1->xpos - current_body2->xpos;
	y_dist = current_body1->ypos - current_body2->ypos;
	distance = sqrt(powl(x_dist, 2) + powl(y_dist, 2));
	accel = CALCACCEL(distance, current_body2->mass);
	x_accel += -1 * accel * (x_dist / distance);
	y_accel += -1 * accel * (y_dist / distance);
	current_body2 = current_body2 -> next;
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

struct body * add_body(long double xpos, long double ypos, long double xvel, long double yvel, long double mass, void * attribs, struct body * parent, struct body * root){
  struct body * object = malloc(sizeof(struct body));
  if(!parent){
    object->xpos = xpos;
    object->ypos = ypos;
    object->xvel = xvel;
    object->yvel = yvel;
  }
  else{
    object->xpos = xpos + parent->xpos;
    object->ypos = ypos + parent->ypos;
    object->xvel = xvel + parent->xvel;
    object->yvel = yvel + parent->yvel;
  }
  object->next = NULL;
  object->attribs = attribs;
  object->mass = mass;
  if(!root)
    return object;
  while(root->next)
    root = root->next;
  root->next = object;
  return object;
}

struct body * delete_body(struct body * object, struct body * root){
  struct body * scratchobj1 = root;
  struct body * scratchobj2;
  if(!root)
    return NULL;
  if(!object){
    delete_body(NULL, root->next);
    free(root);
    return NULL;
  }
  if(object == root){
    scratchobj1 = root->next;
    free(object);
    return scratchobj1;
  }
  while(scratchobj1->next != object && !scratchobj1){
    scratchobj1 = scratchobj1->next;
  }
  scratchobj2 = scratchobj1->next->next;
  free(scratchobj1->next);
  scratchobj1->next = scratchobj2;
  return root;
}
