#ifndef LIBSPACETURTLE
#define LIBSPACETURTLE
#include <math.h>

#define G 0.0000000000667408
#define CALCACCEL(d, m) G * m / (d * d)

struct body{
  long double xpos; // meters
  long double ypos; 
  long double xvel; // meters/sec
  long double yvel;
  long double radi;
  long double mass; // kg
  struct body * next;
  unsigned char flags;
  void * attribs; // your application can stick whatever it wants here
  void * privattribs;
};

struct qtree{
  struct qtree * next_node[4];
  struct qtree * parent;
  struct body * cbody;
  long double x;
  long double y;
  long double w;
  long double h;
  long double cmx;
  long double cmy;
  long double mass;
};

struct octree{
  struct octree * next_node[8];
  struct octree * parent;
  struct body3 * cbody;
  long double cmx;
  long double cmy;
  long double cmz;
  long double mass;
};

struct body3{
  long double xpos;
  long double ypos;
  long double zpos;
  long double xvel;
  long double yvel;
  long double zvel;
  long double radi;
  long double mass;
  struct body3 * next;
  unsigned char flags;
  void * attribs;
  void * privattribs;
};

struct world_config{
  long double tickspersec;
};

int step_forward(struct body * root, struct world_config config, unsigned long long int ticks);
int step_forward3(struct body3 * root, struct world_config config, unsigned long long int ticks);

struct body * add_body(long double xpos, long double ypos, long double xvel, long double yvel, long double mass,
		       long double radi, void * attribs, struct body * parent, struct body * root);
struct body3 * add_body3(long double xpos, long double ypos, long double zpos, long double xvel, long double yvel, long double zvel,
			 long double mass, long double radi, void * attribs, struct body3 * parent, struct body3 * root);
struct qtree * create_qtree(struct body * root);
//struct octree * create_octree(struct body3 * root);

struct body * delete_body(struct body * object, struct body * root);
struct body3 * delete_body3(struct body3 * object, struct body3 * root);

#endif
