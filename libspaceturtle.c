#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "libspaceturtle.h"

int step_forward(struct body * root, struct world_config config, unsigned long long int ticks){
  struct body * current_body1;
  struct body * current_body2;
  double x_accel, y_accel, accel, x_dist, y_dist, distance, dvx, dvy;
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
      dvx = x_accel / config.tickspersec;
      dvy = y_accel / config.tickspersec;
      current_body1->xpos += (current_body1->xvel + (dvx / 2)) / config.tickspersec;
      current_body1->ypos += (current_body1->yvel + (dvy / 2)) / config.tickspersec;
      current_body1->xvel += dvx;
      current_body1->yvel += dvy;
      current_body1 = current_body1->next;
    }
  }
  return 0;
}

int step_forward3(struct body3 * root, struct world_config config, unsigned long long int ticks){
  struct body3 * current_body1;
  struct body3 * current_body2;
  double x_accel, y_accel, z_accel, accel, x_dist, y_dist, z_dist, distance, dvx, dvy, dvz;
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
      dvx = x_accel / config.tickspersec;
      dvy = y_accel / config.tickspersec;
      dvz = z_accel / config.tickspersec;
      current_body1->xpos += (current_body1->xvel + (dvx / 2)) / config.tickspersec;
      current_body1->ypos += (current_body1->yvel + (dvy / 2)) / config.tickspersec;
      current_body1->zpos += (current_body1->zvel + (dvz / 2)) / config.tickspersec;
      current_body1->xvel += dvx;
      current_body1->yvel += dvy;
      current_body1->zvel += dvz;
      current_body1 = current_body1->next;
    }
  }
  return 0;
}

int step_forward_tctd(struct body * root, struct world_config config, unsigned long long ticks){
  struct qtree * scratch1, * scratch2, * root_node;
  struct body * scratchb1;
  long double x_dist, y_dist, x_accel, y_accel, distance, accel;
  root_node = create_qtree(root);
  for(unsigned long long int i = 0; i < ticks; i++){
    scratchb1 = root;
    while(scratchb1){
      scratch1 = scratchb1->privattribs;
      scratch2 = scratch1->parent;
      x_accel = 0;
      y_accel = 0;
      while(scratch2){
	for(int p = 0; p <= 3; p++){
	  if(scratch2->next_node[p] != scratch1){
	    x_dist = scratchb1->xpos - scratch2->next_node[p]->cmx;
	    y_dist = scratchb1->ypos - scratch2->next_node[p]->cmy;
	    distance = sqrt(powl(x_dist, 2) + powl(y_dist, 2));
	    accel = CALCACCEL(distance, scratch2->next_node[p]->mass);
	    x_accel += -1 * accel * (x_dist / distance);
	    y_accel += -1 * accel * (y_dist / distance);
	  }
	}
	scratch1 = scratch2;
	scratch2 = scratch1->parent;
      }
      scratchb1->xpos += scratchb1->xvel / config.tickspersec + 0.5 * x_accel * powl(config.tickspersec, -2);
      scratchb1->ypos += scratchb1->yvel / config.tickspersec + 0.5 * y_accel * powl(config.tickspersec, -2);
      scratchb1->xvel += scratchb1->xvel + x_accel / config.tickspersec;
      scratchb1->yvel += scratchb1->yvel + y_accel / config.tickspersec;
      scratchb1 = scratchb1->next;
    }
    delete_qtree(root_node);
    root_node = create_qtree(root);
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
    tree = malloc(sizeof(struct qtree)); // this malloc and the coresponding free are going to be expensive, try to reuse memory.
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
    scratch1->privattribs = tree;
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
  maxx = root->xpos + 1;
  minx = root->xpos - 1;
  maxy = root->ypos + 1;
  miny = root->ypos - 1;
  while(scratch){
    if(scratch->xpos > maxx)
      maxx = scratch->xpos + 1;
    else if(scratch->xpos < minx)
      minx = scratch->xpos - 1;
    if(scratch->ypos > maxy)
      maxy = scratch->ypos + 1;
    else if(scratch->ypos < miny)
      miny = scratch->ypos - 1;
    scratch = scratch->next;
  }
  ret = gen_qnode(root, minx, miny, maxx - minx, maxy - miny);
  ret->parent = NULL;
  return ret;
}

int delete_qtree(struct qtree * root){
  for(int i = 0; i <= 3; i++)
    if(root->next_node[i])
      delete_qtree(root->next_node[i]);
  free(root);
  return 0;
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
