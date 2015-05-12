#ifndef MAP_H_INCLUDED
#define MAP_H_INCLUDED
#define F_CPU 14700000UL
#define MAP_SIZE 17
#include <stdlib.h>
#include <util/delay.h>

int x;
int y;
int xdir;
int ydir;


//Origin is the upper left element.
int driveable[MAP_SIZE][MAP_SIZE];
int explored[MAP_SIZE][MAP_SIZE];
int costmap[MAP_SIZE][MAP_SIZE];
int goal[2];
int robotpos[2];
int start[2];

void shift_up();
void shift_right();
void shift_down();
void shift_left();

void update_map();
void update_position();

void init_map();
void print_drivable();
void print_explored();
void print_costmap();
#endif // MAP_H_INCLUDED
