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

int step_forward3(struct body3 * root, struct world_config config, unsigned long long int ticks){
  struct body3 * current_body1;
  struct body3 * current_body2;
  double x_accel, y_accel, z_accel, accel, x_dist, y_dist, z_dist, distance;
  if(!root)
    return -1;
  for(unsigned long long int i = 0; i < ticks; i++){
    current_body1 = root;
    while(current_body1 != NULL){
      x_accel = 0;
      y_accel = 0;
      z_accel = 0;
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
	z_dist = current_body1->zpos - current_body2->zpos;
	distance = sqrt(powl(x_dist, 2) + powl(y_dist, 2) + powl(z_dist, 2));
	accel = CALCACCEL(distance, current_body2->mass);
	x_accel += -1 * accel * (x_dist / distance);
	y_accel += -1 * accel * (y_dist / distance);
	z_accel += -1 * accel * (z_dist / distance);
	current_body2 = current_body2 -> next;
      }
      current_body1->xvel += x_accel / config.tickspersec;
      current_body1->yvel += y_accel / config.tickspersec;
      current_body1->zvel += z_accel / config.tickspersec;
      current_body1->xpos += current_body1->xvel / config.tickspersec;
      current_body1->ypos += current_body1->yvel / config.tickspersec;
      current_body1->zpos += current_body1->zvel / config.tickspersec;
      current_body1 = current_body1->next;
    }
  }
  return 0;
}

struct body * add_body(long double xpos, long double ypos, long double xvel, long double yvel, long double mass,
		       long double radi, void * attribs, struct body * parent, struct body * root){
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
  object->radi = radi;
  if(!root)
    return object;
  while(root->next)
    root = root->next;
  root->next = object;
  return object;
}

struct body3 * add_body3(long double xpos, long double ypos, long double zpos, long double xvel, long double yvel, long double zvel,
			 long double mass, long double radi, void * attribs, struct body3 * parent, struct body3 * root){
  struct body3 * object = malloc(sizeof(struct body));
  if(!parent){
    object->xpos = xpos;
    object->ypos = ypos;
    object->zpos = zpos;
    object->xvel = xvel;
    object->yvel = yvel;
    object->zvel = zvel;
  }
  else{
    object->xpos = xpos + parent->xpos;
    object->ypos = ypos + parent->ypos;
    object->zpos = zpos + parent->zpos;
    object->xvel = xvel + parent->xvel;
    object->yvel = yvel + parent->yvel;
    object->zvel = zvel + parent->zvel;
  }
  object->next = NULL;
  object->attribs = attribs;
  object->mass = mass;
  object->radi = radi;
  if(!root)
    return object;
  while(root->next)
    root = root->next;
  root->next = object;
  return object;
}

struct qtree * gen_qnode(struct body * root, long double x, long double y, long double w, long double h){
  struct body * scratch1 = root, * scratch2 = root;
  long double cmx = 0, cmy = 0, mass = 0;
  long double contained = 0;
  struct qtree * tree = NULL;
  while(scratch1){
    if((scratch1->xpos > x && scratch1->xpos <= x + w) && (scratch1->ypos > y && scratch1->ypos <= y + h)){
      contained++;
      break;
    }
    scratch1 = scratch1->next;
  }
  if(contained == 1){
    tree = malloc(sizeof(struct qtree));
    tree->next_node[0] = NULL;
    tree->next_node[1] = NULL;
    tree->next_node[2] = NULL;
    tree->next_node[3] = NULL;
    while(scratch2){
      if((scratch2->xpos > x && scratch2->xpos <= x + w) && (scratch2->ypos > y && scratch2->ypos <= y + h)){
	contained++;
	if((scratch2->xpos > x && scratch2->xpos <= x + (w / 2)) && (scratch2->ypos > y && scratch2->ypos <= y + (h / 2))){
	  if(!tree->next_node[0]){
	    tree->next_node[0] = gen_qnode(root, x, y, w / 2, h / 2);
	  }
	}
	if((scratch2->xpos > x + (w / 2) && scratch2->xpos <= x + w) && (scratch2->ypos > y && scratch2->ypos <= y + (h / 2))){
	  if(!tree->next_node[1]){
	    tree->next_node[1] = gen_qnode(root, x + (w / 2), y, w / 2, h / 2);
	  }
	}
	if((scratch2->xpos > x && scratch2->xpos <= x + (w / 2)) && (scratch2->ypos > y + (h / 2) && scratch2->ypos <= y + h)){
	  if(!tree->next_node[2]){
	    tree->next_node[2] = gen_qnode(root, x, y + (h / 2), w / 2, h / 2);
	  }
	}
	if((scratch2->xpos > x + (w / 2) && scratch2->xpos <= x + w) && (scratch2->ypos > y + (h / 2) && scratch2->ypos <= y + h)){
	  if(!tree->next_node[3]){
	    tree->next_node[3] = gen_qnode(root, x + (w / 2), y + (h / 2), w / 2, h / 2);
	  }
	}
      }
    }
  } else
    return NULL;
  if(contained == 1){
    tree->cbody = scratch1;
    tree->cmx = scratch1->xpos;
    tree->cmy = scratch1->ypos;
    tree->mass = scratch1->mass;
    tree->x = x;
    tree->y = y;
    tree->w = w;
    tree->h = h;
    return tree;
  }
  if(tree->next_node[0]){
    cmx += tree->next_node[0]->cmx / contained;
    cmy += tree->next_node[0]->cmy / contained;
    mass += tree->next_node[0]->mass;
    tree->next_node[0]->parent = tree;
  }
  if(tree->next_node[1]){
    cmx += tree->next_node[1]->cmx / contained;
    cmy += tree->next_node[1]->cmy / contained;
    mass += tree->next_node[1]->mass;
    tree->next_node[1]->parent = tree;
  }
  if(tree->next_node[2]){
    cmx += tree->next_node[2]->cmx / contained;
    cmy += tree->next_node[2]->cmy / contained;
    mass += tree->next_node[2]->mass;
    tree->next_node[2]->parent = tree;
  }
  if(tree->next_node[3]){
    cmx += tree->next_node[3]->cmx / contained;
    cmy += tree->next_node[3]->cmy / contained;
    mass += tree->next_node[3]->mass;
    tree->next_node[3]->parent = tree;
  }
  tree->cmx = cmx;
  tree->cmy = cmy;
  tree->mass = mass;
  tree->x = x;
  tree->y = y;
  tree->w = w;
  tree->h = h;
  return tree;
}

struct qtree * create_qtree(struct body * root){
  struct qtree * ret;
  struct body * scratch = root;
  long double maxx, maxy, minx, miny;
  if(!root)
    return NULL;
  maxx = root->xpos;
  minx = root->xpos;
  maxy = root->ypos;
  miny = root->ypos;
  while(scratch){
    if(scratch->xpos > maxx)
      maxx = scratch->xpos;
    else if(scratch->xpos < minx)
      minx = scratch->xpos;
    if(scratch->ypos > maxy)
      maxy = scratch->ypos;
    else if(scratch->ypos < miny)
      miny = scratch->ypos;
    scratch = scratch->next;
  }
  ret = gen_qnode(root, minx, miny, maxx - minx, maxy - miny);
  return ret;
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

struct body3 * delete_body3(struct body3 * object, struct body3 * root){
  struct body3 * scratchobj1 = root;
  struct body3 * scratchobj2;
  if(!root)
    return NULL;
  if(!object){
    delete_body3(NULL, root->next);
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
