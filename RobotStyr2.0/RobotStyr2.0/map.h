#ifndef MAP_H_INCLUDED
#define MAP_H_INCLUDED
#define F_CPU 14700000UL
#define MAP_SIZE 17
#include <stdlib.h>
#include <util/delay.h>

int8_t x;
int8_t y;
int8_t xdir;
int8_t ydir;
int8_t goalx;
int8_t goaly;

//Origin is the upper left element.
int driveable[MAP_SIZE][MAP_SIZE];
int explored[MAP_SIZE][MAP_SIZE];
int costmap[MAP_SIZE][MAP_SIZE];

//int start[2];

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
