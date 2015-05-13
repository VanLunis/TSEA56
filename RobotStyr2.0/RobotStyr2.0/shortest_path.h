#ifndef SHORTEST_PATH_H_INCLUDED
#define SHORTEST_PATH_H_INCLUDED
#include "map.h"

typedef struct point
{
    int x;
    int y;
} point;

unsigned char command[50];
int c;
point end;
point path[50];
point unvisited[50];
int un;

void fill_square(point p, int cost);

void getCommands(point end);

void traceBack(int costmap[17][17], point end);

void floodfill(point start, point end);
#endif
