#ifndef MAP_H_INCLUDED
#define MAP_H_INCLUDED
#define MAP_SIZE 17
#include <stdlib.h>


typedef struct robo{
    int x;
    int y;
    int distance;
    int xdir;
    int ydir;
    int fwall;
    int rwall;
    int lwall;
    int bwall;
} robo;

//Origin is the upper left element.
int driveable[MAP_SIZE][MAP_SIZE];
int explored[MAP_SIZE][MAP_SIZE];
int costmap[MAP_SIZE][MAP_SIZE];
int goal[2];
int robotpos[2];
int start[2];

robo robot;

void shift_up();
void shift_right();
void shift_down();
void shift_left();

void update_map();

void init_map();
void print_drivable();
void print_explored();
void print_costmap();
#endif // MAP_H_INCLUDED
