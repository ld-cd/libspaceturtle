#ifndef LIBSPACETURTLE
#define LIBSPACETURTLE
#define G 0.00000000006674
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

#endif
