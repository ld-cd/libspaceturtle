#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "libspaceturtle.h"

inline void calcaccel(struct body * cb1, struct body * cb2, long double * xa, long double * ya);
inline void calcposndv(struct body * body, long double x_accel, long double y_accel, struct world_config config);

unsigned long long countbodies(struct body * root);

int step_forward(struct body * root, struct world_config config, unsigned long long int ticks){
  struct body * current_body1;
  struct body * current_body2;
  unsigned long long cindex = 0;
  unsigned long long numbodies = countbodies(root);
  long double x_accels[numbodies];
  long double y_accels[numbodies];
  if(!root)
    return -1;
  for(unsigned long long int i = 0; i < ticks; i++){
    current_body1 = root;
    cindex = 0;
    while(current_body1){
      x_accels[cindex] = 0;
      y_accels[cindex] = 0;
      current_body2 = root;
      while(current_body2){
	while((current_body2->next == current_body1->next) || (current_body2->mass == 0.0)){
	  current_body2 = current_body2->next;
	  if(!current_body2)
	    break;
	}
	if(!current_body2)
	  break;
	calcaccel(current_body1, current_body2, &x_accels[cindex], &y_accels[cindex]);
	current_body2 = current_body2 -> next;
      }
      current_body1 = current_body1->next;
      cindex++;
    }
    current_body1 = root;
    cindex = 0;
    while(current_body1){  //having this loop here is both more correct and easier to vectorize than the old way
      calcposndv(current_body1, x_accels[cindex], y_accels[cindex], config);
      current_body1 = current_body1->next;
      cindex++;
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
	distance = SQUARE(x_dist) + SQUARE(y_dist) + SQUARE(z_dist);
	accel = CALCACCEL(distance, current_body2->mass);
	distance = sqrtl(distance);
	x_accel += -1 * accel * (x_dist / distance);
	y_accel += -1 * accel * (y_dist / distance);
	z_accel += -1 * accel * (z_dist / distance);
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
  long double x_dist, y_dist, x_accel, y_accel, distance, accel, dvx, dvy;
  root_node = create_qtree(root, NULL);
  for(unsigned long long int i = 0; i < ticks; i++){
    scratchb1 = root;
    while(scratchb1){
      scratch1 = scratchb1->privattribs;
      scratch2 = scratch1->parent;
      x_accel = 0;
      y_accel = 0;
      while(scratch2){
	for(int p = 0; p <= 3; p++){
	  if(scratch2->next_node[p]){
	    if(scratch2->next_node[p] != scratch1){
	      x_dist = scratchb1->xpos - scratch2->next_node[p]->cmx;
	      y_dist = scratchb1->ypos - scratch2->next_node[p]->cmx;
	      distance = SQUARE(x_dist) + SQUARE(y_dist);
	      accel = CALCACCEL(distance, scratch2->next_node[p]->mass);
	      distance = sqrtl(distance);
	      x_accel += -1 * accel * (x_dist / distance);
	      y_accel += -1 * accel * (y_dist / distance);
	    }
	  }
	}
	scratch1 = scratch2;
	scratch2 = scratch1->parent;
      }
      dvx = x_accel / config.tickspersec;
      dvy = y_accel / config.tickspersec;
      scratchb1->xpos += (scratchb1->xvel + (dvx / 2)) / config.tickspersec;
      scratchb1->ypos += (scratchb1->yvel + (dvy / 2)) / config.tickspersec;
      scratchb1->xvel += dvx;
      scratchb1->yvel += dvy;
      scratchb1 = scratchb1->next;
    }
    root_node = create_qtree(root, root_node);
  }
  delete_qtree(root_node);
  return 0;
}

inline void calcaccel(struct body * cb1, struct body * cb2, long double * xa, long double * ya){
  long double accel, x_dist, y_dist, distance;
  x_dist = cb1->xpos - cb2->xpos;
  y_dist = cb1->ypos - cb2->ypos;
  distance = SQUARE(x_dist) + SQUARE(y_dist);
  accel = CALCACCEL(distance, cb2->mass);
  distance = sqrtl(distance);
  *xa += -1 * accel * (x_dist / distance);
  *ya += -1 * accel * (y_dist / distance);
}

inline void calcposndv(struct body * body, long double x_accel, long double y_accel, struct world_config config){
  long double dvx, dvy;
  dvx = x_accel / config.tickspersec;
  dvy = y_accel / config.tickspersec;
  body->xpos += (body->xvel + (dvx / 2)) / config.tickspersec;
  body->ypos += (body->yvel + (dvy / 2)) / config.tickspersec;
  body->xvel += dvx;
  body->yvel += dvy;
}

unsigned long long countbodies(struct body * root){
  unsigned long long ret = 0;
  while(root){
    ret++;
    root = root->next;
  }
  return ret;
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

struct qtree * gen_qnode(struct body * root, long double x, long double y, long double w, long double h, struct qtree * freemem){
  struct body * scratch1 = root, * scratch2 = root;
  long double cmx = 0, cmy = 0, mass = 0;
  int contained = 0;
  int quads[4];
  struct qtree * tree = NULL;
  while(scratch1){
    if((scratch1->xpos > x && scratch1->xpos <= x + w) && (scratch1->ypos > y && scratch1->ypos <= y + h)){
      contained++;
    }
    scratch1 = scratch1->next;
  }
  if(contained > 1){
    if(freemem)
      tree = freemem;
    else{
      tree = malloc(sizeof(struct qtree)); // this malloc and the coresponding free are going to be expensive, try to reuse memory.
      tree->next_node[0] = NULL;
      tree->next_node[1] = NULL;
      tree->next_node[2] = NULL;
      tree->next_node[3] = NULL;
    }
    quads[0] = 0;quads[1] = 0;quads[2] = 0;quads[3] = 0;
    contained = 0;
    while(scratch2){
      if((scratch2->xpos > x && scratch2->xpos <= x + w) && (scratch2->ypos > y && scratch2->ypos <= y + h)){
	contained++;
	if((scratch2->xpos > x && scratch2->xpos <= x + (w / 2)) && (scratch2->ypos > y && scratch2->ypos <= y + (h / 2))){
	  if(!quads[0]){
	    tree->next_node[0] = gen_qnode(root, x, y, w / 2, h / 2, tree->next_node[0]);
	    quads[0] = 1;
	  }
	}
	if((scratch2->xpos > x + (w / 2) && scratch2->xpos <= x + w) && (scratch2->ypos > y && scratch2->ypos <= y + (h / 2))){
	  if(!quads[1]){
	    tree->next_node[1] = gen_qnode(root, x + (w / 2), y, w / 2, h / 2, tree->next_node[1]);
	    quads[1] = 1;
	  }
	}
	if((scratch2->xpos > x && scratch2->xpos <= x + (w / 2)) && (scratch2->ypos > y + (h / 2) && scratch2->ypos <= y + h)){
	  if(!quads[2]){
	    tree->next_node[2] = gen_qnode(root, x, y + (h / 2), w / 2, h / 2, tree->next_node[2]);
	    quads[2] = 1;
	  }
	}
	if((scratch2->xpos > x + (w / 2) && scratch2->xpos <= x + w) && (scratch2->ypos > y + (h / 2) && scratch2->ypos <= y + h)){
	  if(!quads[3]){
            tree->next_node[3] = gen_qnode(root, x + (w / 2), y + (h / 2), w / 2, h / 2, tree->next_node[3]);
	    quads[3] = 1;
	  }
	}
      }
      scratch2 = scratch2->next;
    }
  } else if(contained == 0){
    if(freemem)
      delete_qtree(freemem);
    return NULL;
  }
  if(contained == 1){
    scratch1 = root;
    while(scratch1){
      if((scratch1->xpos > x && scratch1->xpos <= x + w) && (scratch1->ypos > y && scratch1->ypos <= y + h)){
	break;
      }
      scratch1 = scratch1->next;
    }
    if(freemem){
      tree = freemem;
      delete_qtree(tree->next_node[0]);
      delete_qtree(tree->next_node[1]);
      delete_qtree(tree->next_node[2]);
      delete_qtree(tree->next_node[3]);
    } else
      tree = malloc(sizeof(struct qtree));
    tree->next_node[0] = NULL;
    tree->next_node[1] = NULL;
    tree->next_node[2] = NULL;
    tree->next_node[3] = NULL;
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
  for(int i = 0; i <= 3; i++){
    if(quads[i]){
      cmx += tree->next_node[i]->cmx / contained;
      cmy += tree->next_node[i]->cmy / contained;
      mass += tree->next_node[i]->mass;
      tree->next_node[i]->parent = tree;
    } else if(tree->next_node[i])
      delete_qtree(tree->next_node[i]);
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

struct qtree * create_qtree(struct body * root, struct qtree * root_node){
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
  ret = gen_qnode(root, minx - 0.5, miny - 0.5, maxx + 1 - minx, maxy + 1 - miny, root_node);
  ret->parent = NULL;
  return ret;
}

int delete_qtree(struct qtree * root){
  if(!root)
    return 0;
  for(int i = 0; i <= 3; i++)
    if(root->next_node[i])
      delete_qtree(root->next_node[i]);
  if(root->parent)
    for(int i = 0; i <= 3; i++)
      if(root == root->parent->next_node[i])
	root->parent->next_node[i] = NULL;
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
