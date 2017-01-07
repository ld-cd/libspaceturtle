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
  long double mass; // kg
  struct body * next;
};

struct world_config{
  long double tickspersec;
};

int step_forward(struct body * root, struct world_config config, unsigned long long int ticks);
struct body * add_body(long double xpos, long double ypos, long double xvel, long double yvel, long double mass, struct body * root, struct body * parent);
struct body * delete_body(struct body * object, struct body * root);

#endif
