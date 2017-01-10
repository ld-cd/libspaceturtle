# libspaceturtle

Libspaceturtle is a small n-body simulation program that has both 2d and 3d particle particle integration methods, as well as a two dimensional version of the Barnes-Hut Tree Code algorithim.

## Building

As of yet there are no dependencies, to build the library simply run `make` in the main directory and `make install` to install it.

If you want to see a quick preview run `make test`, and `./test`, this will use both 2d methods to simulate a 10kg body in an orbit similar to that of the ISS.

## Usage

All of the solvers operate on a linked list of bodies (type struct body), create your root node by running:
i
```C
struct body * root = add_body(xposition, yposition, xvelocity, yvelocity, mass, radius, NULL, NULL, NULL); // Units are kg, m, s
```

You can then add further bodies to that list like so:

```C
add_body(xposition, yposition, xvelocity, yvelocity, mass, radius, NULL, NULL, root);

/* the first pointer can be a pointer to an attribute your application requires,

the second can be a pointer to a parent body making position and velocity an offset from that body */
```

To run the simulation pick a solver and do the following:

```C
struct world_config system;

system.tickspersec; // specifys the number of ticks in a second

// add _tctd to use the Barnes-Hut method

step_forward(root, system, 1000); // this runs the simulation forward 1000 ticks;
```

In order to access the results simply read the variables xpos, ypos, xvel, and yvel from the body struct.
